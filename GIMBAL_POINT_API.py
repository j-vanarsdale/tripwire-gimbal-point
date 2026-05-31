#!/usr/bin/env python3
import time, math, signal, sys, argparse, threading, os
from pymavlink import mavutil
import json
from datetime import datetime, timezone
from flask import Flask, request, jsonify
import subprocess
from adsb import TrackList, closest_track

# --------------------
# DEFAULT CONSTANTS
# (set via argparse)
# --------------------
DEFAULT_PORT        = "/dev/ttyAMA0"
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

app = Flask(__name__)

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
            print(f"Connecting to {port} @ {baud}...")
            master = mavutil.mavlink_connection(port, baud=baud)
            master.wait_heartbeat(timeout=5)
            print("Connected.")
            return master
        except Exception:
            print("Connection failed. Retrying in 2 seconds...")
            time.sleep(2)

def get_temp():
    try:
        temp_raw = subprocess.check_output(["vcgencmd", "measure_temp"]).decode()
        return float(temp_raw.split('=')[1].split("'")[0])
    except Exception:
        return None

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
    global targeting_active
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

        targeting_active = True
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
    global targeting_active
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

        targeting_active = False
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
    """Return the latest telemetry payload from the gimbal loop."""
    with state_lock:
        payload = dict(latest_payload)

    if not payload:
        return jsonify({"error": "No telemetry available yet"}), 503

    return jsonify(payload), 200


# --------------------
# GIMBAL LOOP
# --------------------

def gimbal_loop(args, fallback, tracks=None):
    global latest_payload

    alpha = args.smooth

    # ---- Telemetry JSONL file state ----
    telemetry_file = None
    last_rotation_time = time.time()

    master   = connect_with_retry(args.port, args.baud)
    last_raw = None
    smooth_pos = None
    heading    = 0.0
    roll_meas  = pitch_meas = yaw_meas = 0.0
    live = 0
    hdop = 0
    cube_time = 0
    temp = 0
    telem_fields = ['GLOBAL_POSITION_INT', 'GIMBAL_DEVICE_ATTITUDE_STATUS']
    adsb_log_file = None 

    def cleanup(*_):
        print("Closing connection...")
        try:
            master.close()
        except Exception:
            pass
        if tracks is not None:
            tracks.stop()
        if telemetry_file:
            telemetry_file.close()
        if adsb_log_file:              # add this
            adsb_log_file.close()
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
            pmsg = master.messages['GPS_RAW_INT']
            tmsg = master.messages['SCALED_PRESSURE']
            

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

        if pmsg:
            hdop = pmsg.eph
            cube_time = pmsg.time_usec
        if tmsg:
            temp = tmsg.temperature / 100.0


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

        # ---- ADS-B tracking mode ----
        # Runs independently of targeting_active — updates the target
        # dict which the existing targeting block then uses as normal
        if tracks is not None:
            track = closest_track(tracks, lat, lon)
            
            if track is not None and track.has_position() and track.altitude is not None:
                with state_lock:
                    target["lat"] = track.lat
                    target["lon"] = track.lon
                    target["alt"] = track.altitude * 0.3048   # ft → metres
                    targeting_active = True

                # ADS-B log
                if args.telemetry_dir:
                    record = {
                        "t":          time.time(),
                        "icao":       track.icao,
                        "callsign":   track.callsign,
                        "age":        round(track.age(), 2),
                        "raw_lat":    track.lat,
                        "raw_lon":    track.lon,
                        "raw_alt":    track.altitude,
                    }
                    if adsb_log_file is None:
                        os.makedirs(args.telemetry_dir, exist_ok=True)    # add this
                        ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
                        adsb_log_file = open(
                            os.path.join(args.telemetry_dir, f"{ts}_adsb.jsonl"), "a"
                        )
                    adsb_log_file.write(json.dumps(record) + "\n")
                    adsb_log_file.flush()

        # ---- Targeting mode: recompute pan/tilt from lat/lon every tick ----
        # This runs inside the loop so angles stay correct as the vehicle moves.
        # Targeting is always earth frame; frame flag is forced accordingly.
        with state_lock:
            if targeting_active:
                tgt_lat = target["lat"]
                tgt_lon = target["lon"]
                tgt_alt = target["alt"]

                pitch_calc, yaw_calc = compute_angles(lat, lon, alt, tgt_lat, tgt_lon, tgt_alt)
                pantilt["pan"]   = (yaw_calc   + args.yaw_offset   + 180) % 360 - 180
                pantilt["tilt"]  = (pitch_calc + args.pitch_offset  + 180) % 360 - 180
                pantilt["frame"] = False   # targeting is always earth frame

            pan_out           = pantilt["pan"]
            tilt_out          = pantilt["tilt"]
            use_vehicle_frame = pantilt["frame"]
            mode              = "targeting" if targeting_active else "rc"

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
            "HDOP":       hdop,
            "cube_time":  cube_time,
            "cube_temp": temp,
            "pi_temp": get_temp()
        }

        with state_lock:
            latest_payload = payload

        # ---- Telemetry JSONL logging ----
        if args.telemetry_dir:
            now_t = time.time()
            if telemetry_file is None:
                os.makedirs(args.telemetry_dir, exist_ok=True)
                ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
                filepath = os.path.join(args.telemetry_dir, f"{ts}_telemetry.jsonl")
                telemetry_file = open(filepath, "a")
                last_rotation_time = now_t
            telemetry_file.write(json.dumps(payload) + "\n")
            telemetry_file.flush()
            if now_t - last_rotation_time >= args.telemetry_interval:
                telemetry_file.close()
                telemetry_file = None

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
    parser = argparse.ArgumentParser(description="Gimbal pointer with REST API")
    parser.add_argument("--port",         default=DEFAULT_PORT)
    parser.add_argument("--baud",         type=int,   default=DEFAULT_BAUD)
    parser.add_argument("--fallback",     type=str,   help="lat,lon,alt")
    parser.add_argument("--yaw_offset",   type=float, default=DEFAULT_YAW_OFFSET)
    parser.add_argument("--pitch_offset", type=float, default=DEFAULT_PITCH_OFFSET)
    parser.add_argument("--period",       type=float, default=DEFAULT_PERIOD)
    parser.add_argument("--smooth",       type=float, default=DEFAULT_SMOOTH,
                        help="Smoothing factor: 1=fast, 0.5=smooth, 0=hold value")
    parser.add_argument("--api_port",     type=int,   default=DEFAULT_API_PORT,
                        help="Flask API port (default: 5001)")
    parser.add_argument("--telemetry_dir",    type=str,   default=None,
                        help="Directory to store telemetry JSONL files (disabled if not set)")
    parser.add_argument("--telemetry_interval", type=float, default=60.0,
                        help="Interval in seconds between telemetry file rotations (default: 60)")
    parser.add_argument("--adsb_host",   type=str, default=ADSB_ADDR)
    parser.add_argument("--adsb_port",   type=int, default=ADSB_PORT)
    parser.add_argument("--adsb_enable", action="store_true", default=True,
                        help="Enable ADS-B tracking mode")
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

    # Start ADS-B thread if enabled
    tracks = None
    if args.adsb_enable:
        tracks = TrackList(args.adsb_host, args.adsb_port, prune_after=PRUNE_AFTER)
        tracks.start()
        print(f"ADS-B tracking enabled, connecting to {args.adsb_host}:{args.adsb_port}")
        time.sleep(2)   # let it populate

    # Run gimbal loop on the main thread
    gimbal_loop(args, fallback, tracks)


if __name__ == "__main__":
    main()