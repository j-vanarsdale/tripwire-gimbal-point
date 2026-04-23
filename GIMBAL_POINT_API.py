#!/usr/bin/env python3
import time, math, signal, sys, argparse, threading
from pymavlink import mavutil
import json
from datetime import datetime
from flask import Flask, request, jsonify

# --------------------
# DEFAULT CONSTANTS
# (set via argparse)
# --------------------
DEFAULT_PORT        = "/dev/ttyAMA0"
DEFAULT_BAUD        = 115200
DEFAULT_FALLBACK    = (-33.76535, 150.74087, 0.0)   # lat, lon, alt
DEFAULT_YAW_OFFSET  = 0.0
DEFAULT_PITCH_OFFSET= 0.0
DEFAULT_RATE        = 10
DEFAULT_SMOOTH      = 0.1   # smoothing factor (0–1)
DEFAULT_API_PORT    = 5001
# --------------------

app = Flask(__name__)

# --------------------
# SHARED STATE
# Protected by state_lock
# --------------------
state_lock = threading.Lock()

# USER INPUTS — updated via PUT /target
target = {
    "lat": -33.76535,
    "lon": 150.64087,
    "alt": 50.0,
}

# API OUTPUT — updated by gimbal loop, read via GET /telemetry
latest_payload = {}
# --------------------


# --------------------
# FLASK ROUTES
# --------------------

@app.route("/target", methods=["PUT"])
def put_target():
    """
    Update the gimbal target position.

    Expected JSON body (all fields optional):
        {
            "lat": <float>,
            "lon": <float>,
            "alt": <float>
        }

    Returns the updated target.
    """
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
        updated = dict(target)

    return jsonify({"status": "ok", "target": updated}), 200


@app.route("/target", methods=["GET"])
def get_target():
    """Return the current target position."""
    with state_lock:
        current = dict(target)
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
# HELPERS
# --------------------

def compute_angles(lat0, lon0, alt0, lat1, lon1, alt1):
    R = 6378137.0
    dlat = math.radians(lat1 - lat0)
    dlon = math.radians(lon1 - lon0) * math.cos(math.radians(lat0))
    dn, de, du = R * dlat, R * dlon, alt1 - alt0
    bearing = math.degrees(math.atan2(de, dn))
    elev    = math.degrees(math.atan2(du, math.hypot(dn, de)))
    return elev, bearing


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


# --------------------
# GIMBAL LOOP
# --------------------

def gimbal_loop(args, fallback):
    global latest_payload

    dt    = 1.0 / args.rate
    alpha = args.smooth

    master   = connect_with_retry(args.port, args.baud)
    last_raw = None
    smooth_pos = None
    heading    = 0.0
    roll_meas  = pitch_meas = yaw_meas = 0.0
    live = 0

    def cleanup(*_):
        print("Closing connection...")
        try:
            master.close()
        except Exception:
            pass
        sys.exit(0)

    signal.signal(signal.SIGINT,  cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    # Request gimbal attitude at 10 Hz
    print("Initializing gimbal pose telemetry...\n")
    message = master.mav.command_long_encode(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS,
        100000, 0, 0, 0, 0, 0,
    )
    master.mav.send(message)

    response = master.recv_match(type="COMMAND_ACK", blocking=True)
    if (response
            and response.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL
            and response.result  == mavutil.mavlink.MAV_RESULT_ACCEPTED):
        print("Command accepted")
    else:
        print("Command failed or no ACK received")

    print("Running...\n")
    init_run = True

    while True:
        # ---- GPS position ----
        try:
            msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=init_run)
        except Exception:
            print("Lost connection. Reconnecting...")
            master = connect_with_retry(args.port, args.baud)
            continue

        live = 0  # assume fallback until proven otherwise

        if msg:
            if msg.lat != 0:
                last_raw = (msg.lat / 1e7, msg.lon / 1e7, msg.alt / 1000)
                fallback  = last_raw
                live      = 1
            else:
                last_raw  = fallback
            heading = msg.hdg / 100.0

        # ---- Gimbal attitude ----
        try:
            gmsg = master.recv_match(type="GIMBAL_DEVICE_ATTITUDE_STATUS", blocking=init_run)
        except Exception:
            print("Lost connection. Reconnecting...")
            master = connect_with_retry(args.port, args.baud)
            continue

        if gmsg:
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

        # ---- Read target atomically ----
        with state_lock:
            tgt_lat = target["lat"]
            tgt_lon = target["lon"]
            tgt_alt = target["alt"]

        # ---- Angle computation ----
        pitch_calc, yaw_calc = compute_angles(lat, lon, alt, tgt_lat, tgt_lon, tgt_alt)

        yaw_out   = (yaw_calc   - heading + args.yaw_offset   + 180) % 360 - 180
        pitch_out = (pitch_calc            + args.pitch_offset + 180) % 360 - 180

        # ---- API OUTPUT — update shared telemetry ----
        payload = {
            "t":          time.time(),
            "type":       "Telem",
            "lat":        lat,
            "lon":        lon,
            "alt":        alt,
            "heading":    heading,
            "live":       live,
            "tgt_lat":    tgt_lat,
            "tgt_lon":    tgt_lon,
            "tgt_alt":    tgt_alt,
            "yaw_calc":   yaw_calc,
            "yaw_out":    yaw_out,
            "pitch_calc": pitch_calc,
            "roll_meas":  roll_meas,
            "pitch_meas": pitch_meas,
            "yaw_meas":   yaw_meas,
        }

        with state_lock:
            latest_payload = payload

        # ---- Gimbal control ----
        try:
            master.mav.command_long_send(
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
                0,
                pitch_out, 0, yaw_out,
                0, 0, 0, 2,
            )
        except Exception:
            print("Send failure. Reconnecting...")
            master = connect_with_retry(args.port, args.baud)
            continue

        init_run = False
        time.sleep(dt)


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
    parser.add_argument("--rate",         type=int,   default=DEFAULT_RATE)
    parser.add_argument("--smooth",       type=float, default=DEFAULT_SMOOTH,
                        help="Smoothing factor: 1=fast, 0.5=smooth, 0=hold value")
    parser.add_argument("--api_port",     type=int,   default=DEFAULT_API_PORT,
                        help="Flask API port (default: 5000)")
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

    # Run gimbal loop on the main thread
    gimbal_loop(args, fallback)


if __name__ == "__main__":
    main()