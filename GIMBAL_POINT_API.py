#!/usr/bin/env python3
"""
Gimbal control REST API.

Handles lat/lon/alt targeting, direct RC pan/tilt, and track following —
either an ADS-B lock or an arbitrary track polled from a JSON URL — and
drives the gimbal over MAVLink. Telemetry logging and the full-fidelity
telemetry API live in the separate TELEMETRY_API.py service; this service
exposes only cutdown telemetry views with what gimbal control needs
(GET /telemetry, GET /telemetry/position, GET /telemetry/gimbal).
"""
import time, math, signal, sys, argparse, threading, os
from pymavlink import mavutil
from flask import Flask, request, jsonify
from adsb import TrackList, DeadReckonTracker, PolledTrack

# --------------------
# DEFAULT CONSTANTS
# (set via argparse)
# --------------------
DEFAULT_PORT        = "udp:127.0.0.1:14550"
DEFAULT_BAUD        = 115200
DEFAULT_FALLBACK    = (-33.76535, 150.74087, 0.0)   # lat, lon, alt
DEFAULT_YAW_OFFSET  = 0.0
DEFAULT_PITCH_OFFSET= 0.0
DEFAULT_PERIOD        = 2
DEFAULT_SMOOTH      = 0.1   # smoothing factor (0–1)
DEFAULT_API_PORT    = 5001
# --------------------

# MAVLink gimbal frame flags
# MAV_GIMBAL_DEVICE_FLAGS:
#   bit 4 (16) = YAW_LOCK (earth-frame yaw)
#   bit 5 (32) = PITCHYAW_ABSOLUTE  — combined with bit 4 gives 48 (vehicle) or 80 (earth)
FRAME_FLAG_VEHICLE = 0   # pantilt["frame"] == True  → vehicle/body frame
FRAME_FLAG_EARTH   = 16   # pantilt["frame"] == False → earth frame

ADSB_ADDR   = "192.168.40.106"
ADSB_PORT   = 30003
PRUNE_AFTER = 60

DEADZONE = 2.0 # Deadzone for gimbal hysteresis
app = Flask(__name__)

# ── CORS — allow the frontend server (any origin) to call the API ─────────────
# The dashboard HTML is now served from a separate host, so the browser will
# make cross-origin requests to this server.  We allow all origins here because
# the frontend host/port is not known at deploy time.  If you want to restrict
# this, set ALLOWED_ORIGIN in the environment or hardcode it below.
_ALLOWED_ORIGIN = os.environ.get("ALLOWED_ORIGIN", "*")

@app.after_request
def add_cors(response):
    response.headers["Access-Control-Allow-Origin"]  = _ALLOWED_ORIGIN
    response.headers["Access-Control-Allow-Methods"] = "GET, POST, PUT, DELETE, OPTIONS"
    response.headers["Access-Control-Allow-Headers"] = "Content-Type"
    return response

@app.route("/", defaults={"path": ""}, methods=["OPTIONS"])
@app.route("/<path:path>",             methods=["OPTIONS"])
def cors_preflight(path=""):
    resp = app.make_default_options_response()
    resp.headers["Access-Control-Allow-Origin"]  = _ALLOWED_ORIGIN
    resp.headers["Access-Control-Allow-Methods"] = "GET, POST, PUT, DELETE, OPTIONS"
    resp.headers["Access-Control-Allow-Headers"] = "Content-Type"
    return resp

# --------------------
# SHARED STATE
# Protected by state_lock
# --------------------
state_lock = threading.Lock()

# USER INPUTS — target updated via PUT /target (triggers angle recompute)
#               pantilt updated directly via PUT /rc, or indirectly via PUT /target
target = {
    "lat": -33.66535,
    "lon": 150.74087,
    "alt": 50.0,
}

# pantilt is the single source of truth for the gimbal loop.
# "pan"   — yaw angle (degrees)
# "tilt"  — pitch angle (degrees)
# "frame" — False = earth frame (flag 80), True = vehicle/body frame (flag 48)
pantilt = {
    "pan":   0.0,
    "tilt":  0.0,
    "frame": False,
}

# API OUTPUT — updated by gimbal loop, read via GET /telemetry
latest_payload = {}

# Mode flag — True = target lat/lon/alt tracking mode (loop recomputes angles every tick)
#              False = RC mode (loop uses pantilt values as-is)
targeting_active = False
desired_lead_time_s = 1.0
desired_alt_offset_m = 0.0
desired_yaw_offset_deg = 0.0
adsb_target_icao = None   # set via API, None = not tracking
ext_track = None          # PolledTrack while URL tracking is active, None otherwise
tracks = None
dr_tracker = None
# --------------------


# --------------------
# HELPERS
# --------------------

def compute_angles(lat0, lon0, alt0, lat1, lon1, alt1):
    R = 6378137.0
    dlat = math.radians(lat1 - lat0)
    dlon = math.radians(lon1 - lon0) * math.cos(math.radians(lat0))
    dn, de, du = R * dlat, R * dlon, alt1 - alt0
    bearing = math.degrees(math.atan2(de, dn))
    elev    = math.degrees(math.atan2(du, math.hypot(dn, de)))
    return elev, bearing   # (pitch, yaw)


def quaternion_to_euler(w, x, y, z):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = max(-1.0, min(1.0, t2))
    pitch = math.degrees(math.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.degrees(math.atan2(t3, t4))
    return roll, pitch, yaw


def connect_with_retry(port, baud):
    while True:
        try:
            if port.startswith("udp"):
                print(f"Connecting to UDP {port}...")
                master = mavutil.mavlink_connection(port)
            else:
                print(f"Connecting to {port} @ {baud}...")
                master = mavutil.mavlink_connection(port, baud=baud)
            master.wait_heartbeat(timeout=5)
            print("Connected.")
            return master
        except Exception:
            print("Connection failed. Retrying in 2 seconds...")
            time.sleep(2)

def _stop_ext_track():
    """Stop and clear the polled URL track, if any. Call with state_lock held."""
    global ext_track
    old = ext_track
    ext_track = None
    if old is not None:
        old.stop()

# --------------------
# FLASK ROUTES
# --------------------


@app.route("/target", methods=["PUT"])
def put_target():
    """
    Update the gimbal target position and activate targeting mode.
    The gimbal loop will recompute pan/tilt angles from the target every tick,
    tracking the vehicle as it moves. Frame is automatically set to earth frame.

    Expected JSON body (all fields optional):
        {
            "lat": <float>,
            "lon": <float>,
            "alt": <float>
        }

    Returns the updated target.
    """
    global adsb_target_icao, targeting_active
    data = request.get_json(force=True, silent=True)
    if not data:
        return jsonify({"error": "Invalid or missing JSON body"}), 400

    allowed = {"lat", "lon", "alt"}
    unknown = set(data.keys()) - allowed
    if unknown:
        return jsonify({"error": f"Unknown fields: {unknown}"}), 400

    with state_lock:
        for key in allowed:
            if key in data:
                try:
                    target[key] = float(data[key])
                except (TypeError, ValueError):
                    return jsonify({"error": f"Field '{key}' must be a number"}), 400

        _stop_ext_track()
        targeting_active = True
        adsb_target_icao = None
        updated_target = dict(target)

    return jsonify({
        "status":  "ok",
        "target":  updated_target,
        "mode":    "targeting",
    }), 200


@app.route("/target", methods=["GET"])
def get_target():
    """Return the current target position."""
    with state_lock:
        current = dict(target)
    return jsonify(current), 200


@app.route("/rc", methods=["PUT"])
def put_pantilt():
    """
    Directly set gimbal pan/tilt angles and/or frame of reference.
    Deactivates targeting mode — angles will no longer be recomputed from lat/lon.
    Call PUT /target to re-enable targeting mode.

    Expected JSON body (all fields optional):
        {
            "pan":   <float>,          # yaw angle in degrees
            "tilt":  <float>,          # pitch angle in degrees
            "frame": <bool>            # false = earth frame (flag 80),
                                       # true  = vehicle/body frame (flag 48)
        }

    Returns the updated pantilt state.
    """
    global adsb_target_icao, targeting_active
    data = request.get_json(force=True, silent=True)
    if not data:
        return jsonify({"error": "Invalid or missing JSON body"}), 400

    allowed = {"pan", "tilt", "frame"}
    unknown = set(data.keys()) - allowed
    if unknown:
        return jsonify({"error": f"Unknown fields: {unknown}"}), 400

    with state_lock:
        if "pan" in data:
            try:
                pantilt["pan"] = float(data["pan"])
            except (TypeError, ValueError):
                return jsonify({"error": "Field 'pan' must be a number"}), 400

        if "tilt" in data:
            try:
                pantilt["tilt"] = float(data["tilt"])
            except (TypeError, ValueError):
                return jsonify({"error": "Field 'tilt' must be a number"}), 400

        if "frame" in data:
            if not isinstance(data["frame"], bool):
                return jsonify({"error": "Field 'frame' must be a boolean"}), 400
            pantilt["frame"] = data["frame"]

        _stop_ext_track()
        targeting_active = False
        adsb_target_icao = None
        updated = dict(pantilt)

    return jsonify({"status": "ok", "pantilt": updated, "mode": "rc"}), 200


@app.route("/rc", methods=["GET"])
def get_pantilt():
    """Return the current pan/tilt state."""
    with state_lock:
        current = dict(pantilt)
    return jsonify(current), 200


@app.route("/telemetry", methods=["GET"])
def get_telemetry():
    """
    Return the latest telemetry payload from the gimbal loop.

    This is a cutdown view carrying only what gimbal control needs. Full
    platform telemetry (HDOP, temperatures, raw MAVLink messages) is served
    by the separate telemetry service (TELEMETRY_API.py).
    """
    with state_lock:
        payload = dict(latest_payload)

    if not payload:
        return jsonify({"error": "No telemetry available yet"}), 503

    return jsonify(payload), 200


@app.route("/telemetry/position", methods=["GET"])
def get_telemetry_position():
    """Return just the platform position block of the latest telemetry."""
    with state_lock:
        payload = dict(latest_payload)

    if not payload:
        return jsonify({"error": "No telemetry available yet"}), 503

    keys = ("t", "lat", "lon", "alt", "heading", "live")
    return jsonify({k: payload.get(k) for k in keys}), 200


@app.route("/telemetry/gimbal", methods=["GET"])
def get_telemetry_gimbal():
    """Return just the gimbal output block of the latest telemetry."""
    with state_lock:
        payload = dict(latest_payload)

    if not payload:
        return jsonify({"error": "No telemetry available yet"}), 503

    keys = ("pan_out", "tilt_out", "frame_flag", "mode", "target",
            "roll_meas", "pitch_meas", "yaw_meas")
    return jsonify({k: payload.get(k) for k in keys}), 200


@app.route("/adsb/tracks", methods=["GET"])
def get_adsb_tracks():
    """
    Return all currently visible aircraft as a list sorted by distance
    from the current vehicle position.
    """
    if tracks is None:
        return jsonify({"error": "ADS-B not ready"}), 503

    snapshot = tracks.get_all()

    with state_lock:
        obs_lat = latest_payload.get("lat")
        obs_lon = latest_payload.get("lon")

    def _haversine(lat1, lon1, lat2, lon2):
        R = 6371000.0
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi    = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

    result = []
    for icao, t in snapshot.items():
        if not t.has_position() or t.altitude is None:
            continue
        dist = None
        if obs_lat and obs_lon:
            dist = round(_haversine(obs_lat, obs_lon, t.lat, t.lon))
        result.append({
            "icao":       t.icao,
            "callsign":   t.callsign,
            "lat":        t.lat,
            "lon":        t.lon,
            "alt_ft":     t.altitude,
            "speed":      t.speed,
            "track":      t.track,
            "age":        round(t.age(), 1),
            "distance_m": dist,
        })

    result.sort(key=lambda x: x["distance_m"] if x["distance_m"] is not None else float("inf"))
    return jsonify(result), 200


@app.route("/adsb/track", methods=["PUT"])
def put_adsb_track():
    """
    Lock onto a specific aircraft by ICAO.
    Switches mode to 'adsb', clears any existing targeting, rc or URL track.

    Body: {"icao": "7C1234"}
    """
    global adsb_target_icao, targeting_active
    data = request.get_json(force=True, silent=True)
    if not data or "icao" not in data:
        return jsonify({"error": "Missing field: icao"}), 400

    icao = data["icao"].strip().upper()

    if tracks is None:
        return jsonify({"error": "ADS-B not ready"}), 503

    if icao not in tracks:
        return jsonify({"error": f"ICAO {icao} not currently visible"}), 404

    with state_lock:
        _stop_ext_track()
        adsb_target_icao = icao
        targeting_active = False

    return jsonify({"status": "ok", "mode": "adsb", "icao": icao}), 200

@app.route("/adsb/track", methods=["DELETE"])
def delete_adsb_track():
    """Stop ADS-B tracking and return to rc mode."""
    global adsb_target_icao, targeting_active
    with state_lock:
        adsb_target_icao = None
        targeting_active = False
    return jsonify({"status": "ok", "mode": "rc"}), 200

@app.route("/adsb/lead_time", methods=["PUT"])
def put_lead_time():
    global desired_lead_time_s, dr_tracker
    data = request.get_json(force=True, silent=True)
    if not data or "lead_time_s" not in data:
        return jsonify({"error": "Missing field: lead_time_s"}), 400

    try:
        value = float(data["lead_time_s"])
    except (TypeError, ValueError):
        return jsonify({"error": "lead_time_s must be a number"}), 400

    with state_lock:
        desired_lead_time_s = value
        if dr_tracker is not None:
            dr_tracker.set_lead_time(value)

    return jsonify({"status": "ok", "lead_time_s": value}), 200


@app.route("/adsb/lead_time", methods=["GET"])
def get_lead_time():
    with state_lock:
        value = desired_lead_time_s
    return jsonify({"lead_time_s": value}), 200

@app.route("/adsb/alt_offset", methods=["PUT"])
def put_alt_offset():
    global desired_alt_offset_m
    data = request.get_json(force=True, silent=True)
    if not data or "alt_offset_m" not in data:
        return jsonify({"error": "Missing field: alt_offset_m"}), 400

    try:
        value = float(data["alt_offset_m"])
    except (TypeError, ValueError):
        return jsonify({"error": "alt_offset_m must be a number"}), 400

    with state_lock:
        desired_alt_offset_m = value

    return jsonify({"status": "ok", "alt_offset_m": value}), 200


@app.route("/adsb/alt_offset", methods=["GET"])
def get_alt_offset():
    with state_lock:
        value = desired_alt_offset_m
    return jsonify({"alt_offset_m": value}), 200

@app.route("/adsb/yaw_offset", methods=["PUT"])
def put_yaw_offset():
    global desired_yaw_offset_deg
    data = request.get_json(force=True, silent=True)
    if not data or "yaw_offset_deg" not in data:
        return jsonify({"error": "Missing field: yaw_offset_deg"}), 400

    try:
        value = float(data["yaw_offset_deg"])
    except (TypeError, ValueError):
        return jsonify({"error": "yaw_offset_deg must be a number"}), 400

    with state_lock:
        desired_yaw_offset_deg = value

    return jsonify({"status": "ok", "yaw_offset_deg": value}), 200


@app.route("/adsb/yaw_offset", methods=["GET"])
def get_yaw_offset():
    with state_lock:
        value = desired_yaw_offset_deg
    return jsonify({"yaw_offset_deg": value}), 200

@app.route("/track/url", methods=["PUT"])
def put_url_track():
    """
    Follow an arbitrary track by polling a JSON URL for position updates.
    Switches mode to 'track', clearing any ADS-B lock, targeting or rc mode.
    The same dead-reckoning smoothing and lead/alt/yaw offsets used for
    ADS-B tracking are applied to the polled track.

    Expected JSON body:
        {
            "url":      <string>,          # required, http(s) endpoint
            "interval": <float>,           # poll period in seconds (default 1.0)
            "label":    <string>,          # display name (default: the URL)
            "lat_key":  <string>,          # dot-paths into the JSON object,
            "lon_key":  <string>,          # e.g. "position.lat"
            "alt_key":  <string>,          # (defaults "lat" / "lon" / "alt")
            "speed_key":  <string>,        # optional field overrides; when
            "track_key":  <string>,        # omitted a few common key names
            "vrate_key":  <string>         # are probed automatically
        }

    The JSON must provide lat/lon (degrees) and altitude (metres). Speed
    (m/s), track/heading (deg) and vertical rate (m/s) are optional and
    improve the dead-reckoning when present.
    """
    global ext_track, adsb_target_icao, targeting_active
    data = request.get_json(force=True, silent=True)
    if not data or "url" not in data:
        return jsonify({"error": "Missing field: url"}), 400

    allowed = {"url", "interval", "label",
               "lat_key", "lon_key", "alt_key",
               "speed_key", "track_key", "vrate_key"}
    unknown = set(data.keys()) - allowed
    if unknown:
        return jsonify({"error": f"Unknown fields: {unknown}"}), 400

    url = str(data["url"]).strip()
    if not url.startswith(("http://", "https://")):
        return jsonify({"error": "url must start with http:// or https://"}), 400

    raw_interval = data.get("interval", 1.0)
    if raw_interval is None or raw_interval == "":
        raw_interval = 1.0
    try:
        interval = float(raw_interval)
        if interval <= 0:
            raise ValueError
    except (TypeError, ValueError):
        return jsonify({"error": "interval must be a positive number"}), 400

    # Blank strings (e.g. untouched form fields) count as "not provided"
    kwargs = {}
    for key in ("lat_key", "lon_key", "alt_key", "speed_key", "track_key", "vrate_key"):
        if key in data:
            if not isinstance(data[key], str):
                return jsonify({"error": f"Field '{key}' must be a string"}), 400
            if data[key].strip():
                kwargs[key] = data[key].strip()

    label = data.get("label")
    if label is not None:
        if not isinstance(label, str):
            return jsonify({"error": "Field 'label' must be a string"}), 400
        label = label.strip() or None

    with state_lock:
        _stop_ext_track()
        adsb_target_icao = None
        targeting_active = False
        ext_track = PolledTrack(url, interval=interval, label=label, **kwargs)
        ext_track.start()

    return jsonify({"status": "ok", "mode": "track",
                    "url": url, "interval": interval}), 200


@app.route("/track/url", methods=["GET"])
def get_url_track():
    """Return the status of the polled URL track (active: false when idle)."""
    with state_lock:
        t = ext_track

    if t is None:
        return jsonify({"active": False, "url": None}), 200

    return jsonify({
        "active":   True,
        "url":      t.url,
        "label":    t.callsign,
        "interval": t.interval,
        "age":      round(t.age(), 1),
        "lat":      t.lat,
        "lon":      t.lon,
        "alt_m":    round(t.altitude * 0.3048, 1) if t.altitude is not None else None,
    }), 200


@app.route("/track/url", methods=["DELETE"])
def delete_url_track():
    """Stop URL tracking and return to rc mode."""
    global targeting_active
    with state_lock:
        _stop_ext_track()
        targeting_active = False
    return jsonify({"status": "ok", "mode": "rc"}), 200
# --------------------
# GIMBAL LOOP
# --------------------

def gimbal_loop(args, fallback):
    global latest_payload, targeting_active, dr_tracker

    alpha = args.smooth

    master   = connect_with_retry(args.port, args.baud)
    last_raw = None
    smooth_pos = None
    heading    = 0.0
    roll_meas  = pitch_meas = yaw_meas = 0.0
    live = 0
    telem_fields = ['GLOBAL_POSITION_INT', 'GIMBAL_DEVICE_ATTITUDE_STATUS']
    dr_tracked = None
    track  = None
    pos    = None
    dr_lat = dr_lon = dr_alt = None

    def cleanup(*_):
        print("Closing connection...")
        try:
            master.close()
        except Exception:
            pass
        if tracks is not None:
            tracks.stop()
        if ext_track is not None:
            ext_track.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT,  cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    while True:
        # ---- GPS position ----
        try:
            while master.recv_match(blocking=False):
                pass

            if not all(master.messages.get(name) for name in telem_fields):
                continue
            msg = master.messages['GLOBAL_POSITION_INT']
            gmsg = master.messages['GIMBAL_DEVICE_ATTITUDE_STATUS']

        except Exception:
            print("Lost connection. Reconnecting...")
            master = connect_with_retry(args.port, args.baud)
            continue

        live = 0  # assume fallback until proven otherwise

        if msg.lat != 0:
            last_raw = (msg.lat / 1e7, msg.lon / 1e7, msg.alt / 1000)
            fallback  = last_raw
            live      = 1
        else:
            last_raw  = fallback
        heading = msg.hdg / 100.0

        roll_meas, pitch_meas, yaw_meas = quaternion_to_euler(
            gmsg.q[0], gmsg.q[1], gmsg.q[2], gmsg.q[3]
        )

        # ---- Smoothing ----
        if last_raw is None:
            last_raw = fallback

        if smooth_pos is None:
            smooth_pos = last_raw
        else:
            lat = smooth_pos[0] + alpha * (last_raw[0] - smooth_pos[0])
            lon = smooth_pos[1] + alpha * (last_raw[1] - smooth_pos[1])
            alt = smooth_pos[2] + alpha * (last_raw[2] - smooth_pos[2])
            smooth_pos = (lat, lon, alt)

        lat, lon, alt = smooth_pos

        # ---- Track following (ADS-B lock or polled URL track) ----
        with state_lock:
            icao_to_track = adsb_target_icao
            polled_track  = ext_track

        track = None
        if icao_to_track is not None and tracks is not None:
            track = tracks.get(icao_to_track)
        elif polled_track is not None:
            track = polled_track

        if track is not None and track.has_position() and track.altitude is not None:

            # Reset tracker if the followed track changed
            if track is not dr_tracked:
                dr_tracker = DeadReckonTracker(track, lead_s=desired_lead_time_s)
                dr_tracked = track
                print(f"[Track] Following {track.icao or track.callsign}")

            dr_tracker.step()
            pos = dr_tracker.get()
            if pos:
                dr_lat, dr_lon, dr_alt = pos
                with state_lock:
                    target["lat"] = dr_lat
                    target["lon"] = dr_lon
                    target["alt"] = (dr_alt * 0.3048) + desired_alt_offset_m
                    targeting_active = True
        elif track is None:
            pos = None

        # ---- Targeting mode: recompute pan/tilt from lat/lon every tick ----
        # This runs inside the loop so angles stay correct as the vehicle moves.
        # Targeting is always earth frame; frame flag is forced accordingly.
        with state_lock:
            if targeting_active:
                tgt_lat = target["lat"]
                tgt_lon = target["lon"]
                tgt_alt = target["alt"]
                # API yaw offset (compass correction) applies to track
                # following only, same as lead_time and alt_offset.
                following = adsb_target_icao is not None or ext_track is not None
                yaw_off = desired_yaw_offset_deg if following else 0.0

                pitch_calc, yaw_calc = compute_angles(lat, lon, alt, tgt_lat, tgt_lon, tgt_alt)

                wrapped = (yaw_calc + args.yaw_offset + yaw_off + 180) % 360 - 180
                if abs(wrapped) > (180 - DEADZONE):
                    current_pan = pantilt["pan"]
                    if current_pan >= 0:
                        wrapped = 180 - DEADZONE
                    else:
                        wrapped = -(180 - DEADZONE)
                pantilt["pan"]   = wrapped
                pantilt["tilt"]  = (pitch_calc + args.pitch_offset  + 180) % 360 - 180
                pantilt["frame"] = False   # targeting is always earth frame

            pan_out           = pantilt["pan"]
            tilt_out          = pantilt["tilt"]
            use_vehicle_frame = pantilt["frame"]
            # Mode string — a followed track takes priority over generic targeting
            if adsb_target_icao is not None:
                mode = "adsb"
            elif ext_track is not None:
                mode = "track"
            elif targeting_active:
                mode = "targeting"
            else:
                mode = "rc"

        frame_flag = FRAME_FLAG_VEHICLE if use_vehicle_frame else FRAME_FLAG_EARTH

        # ---- API OUTPUT — update shared telemetry ----
        with state_lock:
            telem_target = dict(target) if targeting_active else None

        payload = {
            "t":          time.time(),
            "type":       "Telem",
            "lat":        lat,
            "lon":        lon,
            "alt":        alt,
            "heading":    heading,
            "live":       live,
            "target":     telem_target,
            "pan_out":    pan_out,
            "tilt_out":   tilt_out,
            "frame_flag": frame_flag,
            "mode":       mode,
            "roll_meas":  roll_meas,
            "pitch_meas": pitch_meas,
            "yaw_meas":   yaw_meas,
            # ---- Followed-track state (None when not tracking) ----
            "adsb_icao":     track.icao     if track else None,
            "adsb_callsign": track.callsign if track else None,
            "adsb_dr_lat":   dr_lat         if pos else None,
            "adsb_dr_lon":   dr_lon         if pos else None,
            "adsb_dr_alt":   dr_alt         if pos else None,
        }

        with state_lock:
            latest_payload = payload

        # ---- Gimbal control ----
        try:
            master.mav.command_long_send(
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
                0,
                tilt_out,   # pitch
                pan_out,    # yaw
                1,          # pitch rate deg/s
                1,          # yaw rate deg/s
                frame_flag, # 48 = vehicle frame, 80 = earth frame
                0,
                0,
            )
        except Exception:
            print("Send failure. Reconnecting...")
            master = connect_with_retry(args.port, args.baud)
            continue

        time.sleep(args.period)


# --------------------
# ENTRY POINT
# --------------------

def main():
    global tracks

    parser = argparse.ArgumentParser(description="Gimbal pointer with REST API")
    parser.add_argument("--port",         default=DEFAULT_PORT,
                        help="MAVLink connection string (default: udp:127.0.0.1:14550)")
    parser.add_argument("--baud",         type=int,   default=DEFAULT_BAUD,
                        help="Serial baud rate (ignored for UDP connections)")
    parser.add_argument("--fallback",     type=str,   help="lat,lon,alt")
    parser.add_argument("--yaw_offset",   type=float, default=DEFAULT_YAW_OFFSET)
    parser.add_argument("--pitch_offset", type=float, default=DEFAULT_PITCH_OFFSET)
    parser.add_argument("--period",       type=float, default=DEFAULT_PERIOD)
    parser.add_argument("--smooth",       type=float, default=DEFAULT_SMOOTH,
                        help="Smoothing factor: 1=fast, 0.5=smooth, 0=hold value")
    parser.add_argument("--api_port",     type=int,   default=DEFAULT_API_PORT,
                        help="Flask API port (default: 5001)")
    parser.add_argument("--adsb_host",   type=str, default=ADSB_ADDR)
    parser.add_argument("--adsb_port",   type=int, default=ADSB_PORT)
    args = parser.parse_args()

    # Resolve fallback
    if args.fallback:
        fb = args.fallback.split(",")
        fallback = (float(fb[0]), float(fb[1]), float(fb[2]))
    else:
        fallback = DEFAULT_FALLBACK

    # Start Flask in a daemon thread so it exits when the main thread exits
    flask_thread = threading.Thread(
        target=lambda: app.run(host="0.0.0.0", port=args.api_port, use_reloader=False),
        daemon=True,
    )
    flask_thread.start()
    print(f"REST API listening on port {args.api_port}")

    # Start ADS-B thread
    tracks = TrackList(args.adsb_host, args.adsb_port, prune_after=PRUNE_AFTER)
    tracks.start()
    print(f"ADS-B tracking enabled, connecting to {args.adsb_host}:{args.adsb_port}")
    time.sleep(2)   # let it populate

    # Run gimbal loop on the main thread
    gimbal_loop(args, fallback)


if __name__ == "__main__":
    main()
