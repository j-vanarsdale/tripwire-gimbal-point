#!/usr/bin/env python3
import time, math, signal, sys, argparse
from pymavlink import mavutil
import json
import curses
# --------------------
# DEFAULT CONSTANTS
# --------------------
DEFAULT_PORT = "/dev/ttyAMA0"
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

import curses

def curses_main(stdscr):
    curses.curs_set(1)
    stdscr.nodelay(True)

    parser = argparse.ArgumentParser(description="Compact gimbal pointer")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--fallback", type=str, help="lat,lon,alt")
    parser.add_argument("--yaw_offset", type=float, default=DEFAULT_YAW_OFFSET)
    parser.add_argument("--pitch_offset", type=float, default=DEFAULT_PITCH_OFFSET)
    parser.add_argument("--rate", type=int, default=DEFAULT_RATE)
    parser.add_argument("--smooth", type=float, default=DEFAULT_SMOOTH)
    args = parser.parse_args()

    dt = 1.0 / args.rate
    alpha = args.smooth

    # fallback setup
    if args.fallback:
        fb = args.fallback.split(",")
        fallback = (float(fb[0]), float(fb[1]), float(fb[2]))
    else:
        fallback = DEFAULT_FALLBACK

    master = connect_with_retry(args.port, args.baud)

    last_raw = None
    smooth_pos = None
    live = None

    input_str = ""       # 👈 replaces input()
    direction = None     # 👈 last submitted command

    while True:
        stdscr.clear()
        h, w = stdscr.getmaxyx()
        mid = w // 2

        # --------------------
        # MAVLINK READ
        # --------------------
        try:
            msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
        except Exception:
            master = connect_with_retry(args.port, args.baud)
            continue

        if msg is None:
            continue

        live = 0

        if msg.lat != 0:
            last_raw = (msg.lat/1e7, msg.lon/1e7, msg.alt/1000)
            fallback = last_raw
            live = 1
        else:
            last_raw = fallback

        heading = msg.hdg / 100.0

        # smoothing
        if smooth_pos is None:
            smooth_pos = last_raw
        else:
            lat = smooth_pos[0] + alpha * (last_raw[0] - smooth_pos[0])
            lon = smooth_pos[1] + alpha * (last_raw[1] - smooth_pos[1])
            alt = smooth_pos[2] + alpha * (last_raw[2] - smooth_pos[2])
            smooth_pos = (lat, lon, alt)

        lat, lon, alt = smooth_pos

        tgt_lat, tgt_lon, tgt_alt = lat, lon, alt

        # --------------------
        # HANDLE COMMAND (from input buffer)
        # --------------------
        if direction:
            if direction == "N":
                tgt_lat += 0.1
            elif direction == "S":
                tgt_lat -= 0.1
            elif direction == "E":
                tgt_lon += 0.1
            elif direction == "W":
                tgt_lon -= 0.1
            elif direction == "1":
                tgt_lat, tgt_lon = -33.76522600900671, 150.7418995647777
            elif direction == "2":
                tgt_lat, tgt_lon = -33.765408585430926, 150.74102906288317

            direction = None  # reset after use

        pitch, yaw = compute_angles(lat, lon, alt, tgt_lat, tgt_lon, tgt_alt)
        yaw_corrected = ((yaw - heading + 180) % 360 - 180)

        payload = {
            "lat": lat,
            "lon": lon,
            "alt": alt,
            "heading": heading,
            "live": live
        }

        # --------------------
        # DRAW UI
        # --------------------

        # LEFT: input
        stdscr.addstr(0, 0, "Commands (Enter to send, q to quit)")
        stdscr.addstr(h-2, 0, "> " + input_str)

        # RIGHT: telemetry
        stdscr.addstr(0, mid, "Telemetry")
        for i, (k, v) in enumerate(payload.items()):
            stdscr.addstr(1 + i, mid, f"{k}: {v}")

        # --------------------
        # INPUT HANDLING
        # --------------------
        key = stdscr.getch()

        if key == ord('q'):
            break
        elif key in (10, 13):  # Enter
            direction = input_str.strip().upper()
            input_str = ""
        elif key in (127, curses.KEY_BACKSPACE):
            input_str = input_str[:-1]
        elif key != -1 and 32 <= key <= 126:
            input_str += chr(key)

        stdscr.refresh()

        # --------------------
        # SEND MAVLINK
        # --------------------
        try:
            master.mav.command_long_send(
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
                0,
                pitch,
                0,
                yaw_corrected,
                0, 0, 0,
                2
            )
        except Exception:
            master = connect_with_retry(args.port, args.baud)

        time.sleep(dt)


if __name__ == "__main__":
    curses.wrapper(curses_main)