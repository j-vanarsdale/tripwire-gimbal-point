#!/usr/bin/env python3
import time, math, signal, sys, argparse
from pymavlink import mavutil, mavextra
import json
from datetime import datetime
# --------------------
# DEFAULT CONSTANTS
# --------------------
DEFAULT_PORT = "/dev/ttyAMA0" # ttyAMA10
DEFAULT_BAUD = 115200

DEFAULT_FALLBACK = (-33.76535, 150.74087, 0.0)  # lat, lon, alt
DEFAULT_YAW_OFFSET = 0.0
DEFAULT_PITCH_OFFSET = 0.0

DEFAULT_RATE = 10
DEFAULT_SMOOTH = 0.1   # smoothing factor (0–1), smoothed = smoothed + alpha * (new-smoothed)

DEFAULT_MODE = 1
DEFAULT_DISTANCE = 0.1
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
            print("Waiting for heartbeat")
            hb = master.wait_heartbeat(timeout=5)
            if hb is None:
                raise RuntimeError("No heartbeat")
            print("Connected.")
            return master

        except Exception as e:
            print(f"Connection failed. {e}. Retrying in 2 seconds...")
            time.sleep(2)
            try:
                master.close()
            except:
                pass
            

def main():
    parser = argparse.ArgumentParser(description="Compact gimbal pointer")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--fallback", type=str, help="lat,lon,alt")
    parser.add_argument("--yaw_offset", type=float, default=DEFAULT_YAW_OFFSET)
    parser.add_argument("--pitch_offset", type=float, default=DEFAULT_PITCH_OFFSET)
    parser.add_argument("--rate", type=int, default=DEFAULT_RATE)
    parser.add_argument("--smooth", type=float, default=DEFAULT_SMOOTH,
                        help="Smoothing factor: 1=fast, 0.5=smooth, 0=hold value")
    parser.add_argument("--mode", type=int, default=DEFAULT_MODE,
                        help="Run mode: 1=look west, 2=user input")
    parser.add_argument("--distance", type=int, default=DEFAULT_DISTANCE,
                        help="Target distance away (lat/lon degrees)")
    args = parser.parse_args()
    
    dt = 1.0 / args.rate
    alpha = args.smooth
    dist = args.distance

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

    last_raw = None
    smooth_pos = None
    live = None     # Live position(1) or fallback(0)
    logname = datetime.now().strftime("%Y%m%d_%H%M") + ".jsonl"
    direction = None # For direction input

    print("Initializing gimbal pose telemetry...\n")
    
    message = master.mav.command_long_encode(
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,  # Confirmation
                mavutil.mavlink.MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS,  # ID: 285
                10000, # param2: Interval in microseconds
                0, 0, 0, 0, 0        
                )
    master.mav.send(message)

    response = master.recv_match(type='COMMAND_ACK', blocking=True)
    if response and response.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL and response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("Command accepted")
    else:
        print("Command failed")

    t0 = time.time()
    print("Running...\n")
    while True:
        t = time.time() - t0
        # If cube disconnects mid‑run — reconnect
        try:
            msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
        except Exception:
            print("Lost connection. Reconnecting...")
            master = connect_with_retry(args.port, args.baud)
            continue
        
        live = 0    # Assume fallback default

        # Check for GPS and update position
        if msg.lat != 0: #msg.lat != 0
            last_raw = (msg.lat/1e7, msg.lon/1e7, msg.alt/1000)
            fallback = last_raw
            live = 1
        else:
            last_raw = (fallback[0], fallback[1], msg.alt/1000) # CHANGE BACK TO JUST fallback. Without GPS, altitude is still provided.

        heading = msg.hdg / 100.0   # Heading SHOULD always return result

        # Gimbal RPY
        try:
            msg = master.recv_match(type="GIMBAL_DEVICE_ATTITUDE_STATUS", blocking=True)
        except Exception:
            print("Lost connection. Reconnecting...")
            master = connect_with_retry(args.port, args.baud)
            continue
        
        #roll_meas= mavextra.quat_to_euler(msg.q)
        roll_meas, pitch_meas, yaw_meas = quaternion_to_euler(msg.q[0], msg.q[1], msg.q[2], msg.q[3])

        # Smoothing filter
        if smooth_pos is None:
            smooth_pos = last_raw
        else:
            lat = smooth_pos[0] + alpha * (last_raw[0] - smooth_pos[0])
            lon = smooth_pos[1] + alpha * (last_raw[1] - smooth_pos[1])
            alt = smooth_pos[2] + alpha * (last_raw[2] - smooth_pos[2])
            smooth_pos = (lat, lon, alt)

        lat, lon, alt = smooth_pos

        # Set target initially to self
        tgt_lat = float(lat)
        tgt_lon = float(lon)
        tgt_alt = float(alt) # Later to be called by actual target alt

        if args.mode == 2:
            if t < 5:
                tgt_lat = float(lat + dist) # N
            elif 5 < t < 10:
                tgt_lat = float(lat - dist) # S
            elif 10 < t < 15:
                tgt_lon = float(lon + dist) # E
            elif 15 < t < 20:
                tgt_lon = float(lon - dist) # W
            else:
                t0 = time.time()
        elif args.mode == 1:
            tgt_lon = float(lon - dist)
            tgt_alt = float(20)
                
        pitch_calc, yaw_calc = compute_angles(lat, lon, alt, tgt_lat, tgt_lon, tgt_alt)

        yaw_out = ((yaw_calc - heading + args.yaw_offset + 180) % 360 - 180) # Wrap 180
        pitch_out = ((pitch_calc + args.pitch_offset + 180) % 360 - 180) # Wrap 180
        
        # To add: battery information, computed roll
        payload = {
            "t": time.time(),
            "type": "Telem",
            "lat": lat,
            "lon": lon,
            "alt": alt,
            "heading": heading,
            "live": live,
            "tgt_lat": tgt_lat,
            "tgt_lon": tgt_lon,
            "tgt_alt": tgt_alt,
            "yaw_calc": yaw_calc,
            "yaw_out": yaw_out,
            "pitch_calc": pitch_calc,
            "roll_meas": roll_meas,
            "pitch_meas": pitch_meas,
            "yaw_meas": yaw_meas,
        }

        # Debug / Logging
        print(payload)
        with open(logname, "a") as f:
            f.write(json.dumps(payload) + "\n")
        
        #print(master.recv_match(type="GPS_RAW_INT", blocking=True).to_dict()) # Check everything im receiving

        # Gimbal Control
        try:
            master.mav.command_long_send(
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
                0,
                pitch_out,
                yaw_out,
                1, # pitch rate deg/s
                1, # yaw rate deg/s
                0, # 0=bodyframe 16=earthframe
                0,
                0
            )
        except Exception:
            print("Send failure. Reconnecting...")
            master = connect_with_retry(args.port, args.baud)
            continue


        time.sleep(dt)

if __name__ == "__main__":
    main()