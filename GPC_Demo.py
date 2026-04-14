#!/usr/bin/env python3
import time, math, signal, sys, argparse
from pymavlink import mavutil
import json
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
# --------------------

def compute_angles(lat0, lon0, alt0, lat1, lon1, alt1):
    R = 6378137.0
    dlat = math.radians(lat1 - lat0)
    dlon = math.radians(lon1 - lon0) * math.cos(math.radians(lat0))
    dn, de, du = R*dlat, R*dlon, alt1-alt0
    bearing = math.degrees(math.atan2(de, dn))
    elev = math.degrees(math.atan2(du, math.hypot(dn, de)))
    return elev, bearing

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
    parser.add_argument("--rate", type=int, default=DEFAULT_RATE)
    parser.add_argument("--smooth", type=float, default=DEFAULT_SMOOTH,
                        help="Smoothing factor: 1=fast, 0.5=smooth, 0=hold value")
    args = parser.parse_args()

    dt = 1.0 / args.rate
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

    last_raw = None
    smooth_pos = None
    live = None     # Live position(1) or fallback(0)

    counter = 0
    print("Running...\n")
    while True:

        # If cube disconnects mid‑run — reconnect
        try:
            msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
        except Exception:
            print("Lost connection. Reconnecting...")
            master = connect_with_retry(args.port, args.baud)
            continue
        
        # Quick restart if mavlink didnt send anything yet
        if msg is None:
            continue
        
        live = 0    # Assume fallback default

        # Check for GPS and update position
        if msg.lat != 0:
            last_raw = (msg.lat/1e7, msg.lon/1e7, msg.alt/1000)
            fallback = last_raw
            live = 1
        else:
            last_raw = fallback
        
        #if msg:
        #    last_raw = (msg.lat/1e7, msg.lon/1e7, msg.alt/1000)
        #elif last_raw and last_raw[0] == 0:
        #    last_raw = fallback
            
        #fallback = last_raw # Update fallback to last known position
        
        heading = msg.hdg / 100.0   # Heading SHOULD always return result

        # Smoothing filter
        if smooth_pos is None:
            smooth_pos = last_raw
        else:
            lat = smooth_pos[0] + alpha * (last_raw[0] - smooth_pos[0])
            lon = smooth_pos[1] + alpha * (last_raw[1] - smooth_pos[1])
            alt = smooth_pos[2] + alpha * (last_raw[2] - smooth_pos[2])
            smooth_pos = (lat, lon, alt)

        lat, lon, alt = smooth_pos

        #-33.76522600900671, 150.7418995647777
        #-33.765408585430926, 150.74102906288317
        # Set target initially to self
        tgt_lat = float(lat)
        tgt_lon = float(lon)
        tgt_alt = float(alt) # Later to be called by actual target alt

        counter += 1
        if counter == 10:
            tgt_lat = float(-33.76522600900671)
            tgt_lon = float(150.7418995647777)
        elif counter == 40:
            tgt_lat = float(-33.765408585430926)
            tgt_lon = float(150.74102906288317)
        elif counter == 70:
            tgt_lat = float(-33.76522600900671)
            tgt_lon = float(150.7418995647777)
        elif counter == 100:
            tgt_lat = float(-33.765408585430926)
            tgt_lon = float(150.74102906288317)
        # Direction used later for target pointing
            
        pitch, yaw = compute_angles(lat, lon, alt, tgt_lat, tgt_lon, tgt_alt)

        yaw_corrected = ((yaw - heading + 180) % 360 - 180) # Wrap 180

        #print(f"heading: {heading} | lat: {lat:.3f} | lon: {lon:.3f} | Raw yaw: {yaw} | yaw: {yaw_corrected}")

        payload = {
            "t": time.time(),
            "type": "Position",
            "lat": lat,
            "lon": lon,
            "alt": alt,
            "heading": heading,
            "live": live
        }
        print(payload)
        try:
            master.mav.command_long_send(
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
                0,
                pitch,#pitch + args.pitch_offset,
                0,
                yaw_corrected, #Wrap 180
                0, 0, 0,
                2
            )
        except Exception:
            print("Send failure. Reconnecting...")
            master = connect_with_retry(args.port, args.baud)
            continue


        time.sleep(dt)

if __name__ == "__main__":
    main()