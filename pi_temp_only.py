#!/usr/bin/env python3
import time, math, signal, sys, argparse, threading, os, subprocess
from flask import Flask, request, jsonify
from datetime import datetime, timezone
import json

DEFAULT_API_PORT    = 5001

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

latest_payload = {}



@app.route("/telemetry", methods=["GET"])
def get_telemetry():
    """Return the latest telemetry payload from the gimbal loop."""
    with state_lock:
        payload = dict(latest_payload)

    if not payload:
        return jsonify({"error": "No telemetry available yet"}), 503

    return jsonify(payload), 200

def get_temp():
    try:
        temp_raw = subprocess.check_output(["vcgencmd", "measure_temp"]).decode()
        return float(temp_raw.split('=')[1].split("'")[0])
    except Exception:
        return None


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Gimbal pointer with REST API")
    parser.add_argument("--api_port",     type=int,   default=DEFAULT_API_PORT,
                        help="Flask API port (default: 5001)")
    parser.add_argument("--telemetry_dir",    type=str,   default=None,
                        help="Directory to store telemetry JSONL files (disabled if not set)")
    parser.add_argument("--telemetry_interval", type=float, default=60.0,
                        help="Interval in seconds between telemetry file rotations (default: 60)")
    
    args = parser.parse_args()

    # Start Flask in a daemon thread so it exits when the main thread exits
    flask_thread = threading.Thread(
        target=lambda: app.run(host="0.0.0.0", port=args.api_port, use_reloader=False),
        daemon=True,
    )
    flask_thread.start()
    print(f"REST API listening on port {args.api_port}")

    # ---- Telemetry JSONL file state ----
    telemetry_file = None
    last_rotation_time = time.time()

    while True:
        

        payload = {
                "pi_temp": get_temp(),
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

        time.sleep(0.5)