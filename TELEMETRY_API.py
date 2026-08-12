#!/usr/bin/env python3
"""
MAVLink telemetry collector and logger.

Connects to its own MAVProxy output, keeps the latest telemetry as JSON
objects, logs them to rotating JSONL files, and serves them over a
read-only REST API. Gimbal control and ADS-B tracking live in
GIMBAL_POINT_API.py — this service only observes the MAVLink stream.

API (GET only):
    GET /telemetry            latest merged telemetry JSON object
    GET /messages             latest raw message of every type seen, by type
    GET /messages/<MSG_TYPE>  latest raw message of one type (404 if unseen)
"""
import time, math, signal, sys, argparse, threading, os
from pymavlink import mavutil
import json
from datetime import datetime, timezone
from flask import Flask, jsonify
import subprocess

# --------------------
# DEFAULT CONSTANTS
# (set via argparse)
# --------------------
DEFAULT_PORT        = "udp:127.0.0.1:14552"
DEFAULT_BAUD        = 115200
DEFAULT_FALLBACK    = (-33.76535, 150.74087, 0.0)   # lat, lon, alt
DEFAULT_PERIOD      = 2
DEFAULT_SMOOTH      = 0.1   # smoothing factor (0–1)
DEFAULT_API_PORT    = 5002
# --------------------

# Message types collected from the MAVLink stream. The latest raw message of
# each is exposed via GET /messages, and the merged telemetry object below is
# built from them.
MSG_TYPES = [
    "GLOBAL_POSITION_INT",
    "GIMBAL_DEVICE_ATTITUDE_STATUS",
    "GPS_RAW_INT",
    "SCALED_PRESSURE",
]

app = Flask(__name__)

# ── CORS — allow the frontend server (any origin) to call the API ─────────────
_ALLOWED_ORIGIN = os.environ.get("ALLOWED_ORIGIN", "*")

@app.after_request
def add_cors(response):
    response.headers["Access-Control-Allow-Origin"]  = _ALLOWED_ORIGIN
    response.headers["Access-Control-Allow-Methods"] = "GET, OPTIONS"
    response.headers["Access-Control-Allow-Headers"] = "Content-Type"
    return response

@app.route("/", defaults={"path": ""}, methods=["OPTIONS"])
@app.route("/<path:path>",             methods=["OPTIONS"])
def cors_preflight(path=""):
    resp = app.make_default_options_response()
    resp.headers["Access-Control-Allow-Origin"]  = _ALLOWED_ORIGIN
    resp.headers["Access-Control-Allow-Methods"] = "GET, OPTIONS"
    resp.headers["Access-Control-Allow-Headers"] = "Content-Type"
    return resp

# --------------------
# SHARED STATE
# Protected by state_lock
# --------------------
state_lock = threading.Lock()

# Latest merged telemetry object, updated by the collector loop.
latest_payload = {}

# Latest raw MAVLink message of each type, as msg.to_dict() keyed by type.
latest_messages = {}
# --------------------


# --------------------
# HELPERS
# --------------------

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

def get_temp():
    try:
        temp_raw = subprocess.check_output(["vcgencmd", "measure_temp"]).decode()
        return float(temp_raw.split('=')[1].split("'")[0])
    except Exception:
        return None

# --------------------
# FLASK ROUTES
# --------------------

@app.route("/telemetry", methods=["GET"])
def get_telemetry():
    """Return the latest merged telemetry object from the collector loop."""
    with state_lock:
        payload = dict(latest_payload)

    if not payload:
        return jsonify({"error": "No telemetry available yet"}), 503

    return jsonify(payload), 200


@app.route("/messages", methods=["GET"])
def get_messages():
    """Return the latest raw MAVLink message of every type seen so far."""
    with state_lock:
        messages = {k: dict(v) for k, v in latest_messages.items()}

    if not messages:
        return jsonify({"error": "No messages received yet"}), 503

    return jsonify(messages), 200


@app.route("/messages/<msg_type>", methods=["GET"])
def get_message(msg_type):
    """Return the latest raw MAVLink message of one type (e.g. GPS_RAW_INT)."""
    key = msg_type.upper()
    with state_lock:
        message = latest_messages.get(key)

    if message is None:
        return jsonify({"error": f"No {key} message received yet"}), 404

    return jsonify(message), 200

# --------------------
# TELEMETRY LOOP
# --------------------

def telemetry_loop(args, fallback):
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

    def cleanup(*_):
        print("Closing connection...")
        try:
            master.close()
        except Exception:
            pass
        if telemetry_file:
            telemetry_file.close()
        sys.exit(0)

    signal.signal(signal.SIGINT,  cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    while True:
        # ---- GPS position ----
        try:
            while master.recv_match(blocking=False):
                pass

            if not all(master.messages.get(name) for name in MSG_TYPES):
                continue
            msg  = master.messages['GLOBAL_POSITION_INT']
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

        # ---- Latest raw messages (for GET /messages) ----
        with state_lock:
            for name in MSG_TYPES:
                raw = master.messages.get(name)
                if raw is not None:
                    latest_messages[name] = raw.to_dict()

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

        # ---- API OUTPUT — update shared telemetry ----
        payload = {
            "t":          time.time(),
            "type":       "Telem",
            "lat":        lat,
            "lon":        lon,
            "alt":        alt,
            "heading":    heading,
            "live":       live,
            "roll_meas":  roll_meas,
            "pitch_meas": pitch_meas,
            "yaw_meas":   yaw_meas,
            "HDOP":       hdop,
            "cube_time":  cube_time,
            "cube_temp":  temp,
            "pi_temp":    get_temp(),
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

        time.sleep(args.period)


# --------------------
# ENTRY POINT
# --------------------

def main():
    parser = argparse.ArgumentParser(description="MAVLink telemetry logger with REST API")
    parser.add_argument("--port",         default=DEFAULT_PORT,
                        help="MAVLink connection string (default: udp:127.0.0.1:14552)")
    parser.add_argument("--baud",         type=int,   default=DEFAULT_BAUD,
                        help="Serial baud rate (ignored for UDP connections)")
    parser.add_argument("--fallback",     type=str,   help="lat,lon,alt")
    parser.add_argument("--period",       type=float, default=DEFAULT_PERIOD)
    parser.add_argument("--smooth",       type=float, default=DEFAULT_SMOOTH,
                        help="Smoothing factor: 1=fast, 0.5=smooth, 0=hold value")
    parser.add_argument("--api_port",     type=int,   default=DEFAULT_API_PORT,
                        help="Flask API port (default: 5002)")
    parser.add_argument("--telemetry_dir",    type=str,   default=None,
                        help="Directory to store telemetry JSONL files (disabled if not set)")
    parser.add_argument("--telemetry_interval", type=float, default=60.0,
                        help="Interval in seconds between telemetry file rotations (default: 60)")
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

    # Run telemetry loop on the main thread
    telemetry_loop(args, fallback)


if __name__ == "__main__":
    main()
