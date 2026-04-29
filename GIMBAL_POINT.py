#!/usr/bin/env python3
import time, math, signal, sys, argparse
from pymavlink import mavutil
import json
from datetime import datetime
# --------------------
#    USER   INPUTS
# --------------------
TARGET_LAT = -33.76535
TARGET_LON = 150.64087
TARGET_ALT = 50.0
# --------------------

# --------------------
# DEFAULT CONSTANTS
# --------------------
DEFAULT_PORT = "/dev/ttyAMA0"
DEFAULT_BAUD = 115200

DEFAULT_FALLBACK = (-33.76535, 150.74087, 0.0)  # lat, lon, alt 
DEFAULT_YAW_OFFSET = 0.0
DEFAULT_PITCH_OFFSET = 0.0

DEFAULT_PERIOD = 2
DEFAULT_SMOOTH = 0.1   # smoothing factor (0–1), smoothed = smoothed + alpha * (new-smoothed)
# --------------------

def compute_angles(lat0, lon0, alt0, lat1, lon1, alt1):
    R = 6378137.0
    dlat = math.radians(lat1 - lat0)
    dlon = math.radians(lon1 - lon0) * math.cos(math.radians(lat0))
    dn, de, du = R*dlat, R*dlon, alt1-alt0
    bearing = math.degrees(math.atan2(de, dn))
    elev = math.degrees(math.atan2(du, math.hypot(dn, de)))
    return elev, bearing

def quaternion_to_euler(w, x, y, z):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
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

            # Wait for heartbeat
            master.wait_heartbeat(timeout=5)
            print("Connected.")
            return master

        except Exception:
            print("Connection failed. Retrying in 2 seconds...")
            time.sleep(2)

def main():
    parser = argparse.ArgumentParser(description="Compact gimbal pointer")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--fallback", type=str, help="lat,lon,alt")
    parser.add_argument("--yaw_offset", type=float, default=DEFAULT_YAW_OFFSET)
    parser.add_argument("--pitch_offset", type=float, default=DEFAULT_PITCH_OFFSET)
    parser.add_argument("--period", type=int, default=DEFAULT_PERIOD)
    parser.add_argument("--smooth", type=float, default=DEFAULT_SMOOTH,
                        help="Smoothing factor: 1=fast, 0.5=smooth, 0=hold value")

    args = parser.parse_args()
    
    alpha = args.smooth

    # Load fallback override
    if args.fallback:
        fb = args.fallback.split(",")
        fallback = (float(fb[0]), float(fb[1]), float(fb[2]))
    else:
        fallback = DEFAULT_FALLBACK

    # --------------------
    # CONNECT WITH RETRY
    # --------------------
    master = connect_with_retry(args.port, args.baud)

    # Cleanup handler
    def cleanup(*_):
        print("Closing connection...")
        try:
            master.close()
        except:
            pass
        sys.exit(0)

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    # ---- Initial states ----
    last_raw = None
    smooth_pos = None
    live = None     # Live position(1) or fallback(0)
    logname = datetime.now().strftime("%Y%m%d_%H%M") + ".jsonl"
    telem_fields = ['GLOBAL_POSITION_INT', 'GIMBAL_DEVICE_ATTITUDE_STATUS','GPS_RAW_INT','EKF_STATUS_REPORT']

    print("Running...\n")
    while True:
        # ---- Recieve data ----
        try:
            while master.recv_match(blocking=False):
                pass

            if not all(master.messages.get(name) for name in telem_fields):
                continue
            msg = master.messages['GLOBAL_POSITION_INT']
            gmsg = master.messages['GIMBAL_DEVICE_ATTITUDE_STATUS']
            pmsg = master.messages['GPS_RAW_INT']
            emsg = master.messages['EKF_STATUS_REPORT']

        except Exception:
            print("Lost connection. Reconnecting...")
            master = connect_with_retry(args.port, args.baud)
            continue

        # ---- GPS position ----
        live = 0    # Assume fallback default
        #print(emsg.flags)
        if pmsg.eph < 100:
            last_raw = (msg.lat/1e7, msg.lon/1e7, msg.alt/1000)
            fallback = last_raw
            live = 1
        else:
            last_raw = fallback
        heading = msg.hdg / 100.0

        roll_meas, pitch_meas, yaw_meas = quaternion_to_euler(gmsg.q[0], gmsg.q[1], gmsg.q[2], gmsg.q[3])

        # ---- Smoothing ----
        if smooth_pos is None:
            smooth_pos = last_raw
        else:
            lat = smooth_pos[0] + alpha * (last_raw[0] - smooth_pos[0])
            lon = smooth_pos[1] + alpha * (last_raw[1] - smooth_pos[1])
            alt = smooth_pos[2] + alpha * (last_raw[2] - smooth_pos[2])
            smooth_pos = (lat, lon, alt)

        lat, lon, alt = smooth_pos

        # ---- Angle computation ----
        pitch_calc, yaw_calc = compute_angles(lat, lon, alt, TARGET_LAT, TARGET_LON, TARGET_ALT)

        yaw_out = ((yaw_calc + args.yaw_offset + 180) % 360 - 180) # Wrap 180
        pitch_out = ((pitch_calc + args.pitch_offset + 180) % 360 - 180) # Wrap 180

        # ---- update telemetry ----
        payload = {
            "t": time.time(),
            "type": "Telem",
            "lat": lat,
            "lon": lon,
            "alt": alt,
            "heading": heading,
            "live": live,
            "tgt_lat": TARGET_LAT,
            "tgt_lon": TARGET_LON,
            "tgt_alt": TARGET_ALT,
            "yaw_calc": yaw_calc,
            "yaw_out": yaw_out,
            "pitch_calc": pitch_calc,
            "roll_meas": roll_meas,
            "pitch_meas": pitch_meas,
            "yaw_meas": yaw_meas,
            "gimbal_heading":heading+yaw_meas,
        }

        # ---- Logger ----
        print(payload)
        with open(logname, "a") as f:
            f.write(json.dumps(payload) + "\n")
        
        # ---- Gimbal control ----
        try:
            master.mav.command_long_send(
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
                0,
                pitch_out,
                yaw_out,
                1, # pitch rate deg/s
                1, # yaw rate deg/s
                16, # 0=bodyframe 16=earthframe
                0,
                0
            )
        except Exception:
            print("Send failure. Reconnecting...")
            master = connect_with_retry(args.port, args.baud)
            continue

        time.sleep(args.period)

if __name__ == "__main__":
    main()