"""
Request gimbal information via MAVLink and monitor runtime yaw-in-earth-frame support.

Three stages:
  1. Request GIMBAL_MANAGER_INFORMATION (280) — manager-level capabilities & angle limits
  2. Request GIMBAL_DEVICE_INFORMATION  (283) — device-level capabilities, including
     SUPPORTS_YAW_IN_EARTH_FRAME
  3. Monitor GIMBAL_DEVICE_STATUS       (285) — watch the runtime
     CAN_ACCEPT_YAW_IN_EARTH_FRAME flag live

Usage:
    python request_gimbal_manager_info.py [connection_string] [--no-monitor]

Examples:
    python request_gimbal_manager_info.py                           # UDP SITL default
    python request_gimbal_manager_info.py udpin:0.0.0.0:14550
    python request_gimbal_manager_info.py tcp:127.0.0.1:5760
    python request_gimbal_manager_info.py /dev/ttyUSB0,57600
    python request_gimbal_manager_info.py udpin:localhost:14550 --no-monitor
"""

import sys
import time
from pymavlink import mavutil

# ── Configuration ─────────────────────────────────────────────────────────────
args              = sys.argv[1:]
MONITOR           = "--no-monitor" not in args
conn_args         = [a for a in args if not a.startswith("--")]
CONNECTION_STRING = conn_args[0] if conn_args else "COM4"
RESPONSE_TIMEOUT  = 5.0    # seconds to wait for each info message
MONITOR_DURATION  = 10.0   # seconds to watch GIMBAL_DEVICE_STATUS
SOURCE_SYSTEM     = 255
SOURCE_COMPONENT  = 0

# MAVLink message IDs
MSG_ID_GIMBAL_MANAGER_INFORMATION = 280
MSG_ID_GIMBAL_DEVICE_INFORMATION  = 283
MSG_ID_GIMBAL_DEVICE_STATUS       = 285

RAD2DEG = 57.295779513


# ── Helpers ───────────────────────────────────────────────────────────────────

def decode_flags(value: int, flag_map: list) -> str:
    active = [name for bit, name in flag_map if value & bit]
    return ", ".join(active) if active else "none"


# ── Connect ───────────────────────────────────────────────────────────────────

def connect(connection_string: str) -> mavutil.mavfile:
    print(f"[*] Connecting to {connection_string} ...")
    mav = mavutil.mavlink_connection(
        connection_string,
        source_system=SOURCE_SYSTEM,
        source_component=SOURCE_COMPONENT,
    )
    mav.wait_heartbeat(timeout=10)
    print(
        f"[+] Heartbeat received "
        f"(system={mav.target_system}, component={mav.target_component})"
    )
    return mav


# ── Request helpers ───────────────────────────────────────────────────────────

def request_message(mav: mavutil.mavfile, msg_id: int) -> None:
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,           # confirmation
        msg_id,      # param1 – message ID to request
        0, 0, 0, 0, 0, 0,
    )


def wait_for_message(mav: mavutil.mavfile, msg_type: str, timeout: float):
    """Wait for a specific message type; also consume and report COMMAND_ACKs."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = mav.recv_match(
            type=[msg_type, "COMMAND_ACK"],
            blocking=True,
            timeout=1.0,
        )
        if msg is None:
            continue
        if msg.get_type() == "COMMAND_ACK":
            if msg.command == mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE:
                result_str = mavutil.mavlink.enums["MAV_RESULT"][msg.result].description
                print(f"  [~] COMMAND_ACK: result={msg.result} ({result_str})")
                if msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    print("  [!] Command rejected – message may not be supported.")
                    return None
        elif msg.get_type() == msg_type:
            return msg
    print(f"  [!] Timed out after {timeout}s waiting for {msg_type}.")
    return None


# ── Stage 1 – GIMBAL_MANAGER_INFORMATION ─────────────────────────────────────

MANAGER_CAP_FLAGS = [
    (mavutil.mavlink.GIMBAL_MANAGER_CAP_FLAGS_HAS_RETRACT,                 "HAS_RETRACT"),
    (mavutil.mavlink.GIMBAL_MANAGER_CAP_FLAGS_HAS_NEUTRAL,                 "HAS_NEUTRAL"),
    (mavutil.mavlink.GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_AXIS,               "HAS_ROLL_AXIS"),
    (mavutil.mavlink.GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_FOLLOW,             "HAS_ROLL_FOLLOW"),
    (mavutil.mavlink.GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_LOCK,               "HAS_ROLL_LOCK"),
    (mavutil.mavlink.GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS,              "HAS_PITCH_AXIS"),
    (mavutil.mavlink.GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_FOLLOW,            "HAS_PITCH_FOLLOW"),
    (mavutil.mavlink.GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_LOCK,              "HAS_PITCH_LOCK"),
    (mavutil.mavlink.GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS,                "HAS_YAW_AXIS"),
    (mavutil.mavlink.GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_FOLLOW,              "HAS_YAW_FOLLOW"),
    (mavutil.mavlink.GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK,                "HAS_YAW_LOCK"),
    (mavutil.mavlink.GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_INFINITE_YAW,       "SUPPORTS_INFINITE_YAW"),
    (mavutil.mavlink.GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_YAW_IN_EARTH_FRAME, "SUPPORTS_YAW_IN_EARTH_FRAME"),
    (mavutil.mavlink.GIMBAL_MANAGER_CAP_FLAGS_HAS_RC_INPUTS,               "HAS_RC_INPUTS"),
]


def fetch_gimbal_manager_information(mav: mavutil.mavfile) -> None:
    print(f"\n{'─'*60}")
    print(f"[1/3] GIMBAL_MANAGER_INFORMATION (msg_id={MSG_ID_GIMBAL_MANAGER_INFORMATION})")
    print(f"{'─'*60}")
    request_message(mav, MSG_ID_GIMBAL_MANAGER_INFORMATION)
    msg = wait_for_message(mav, "GIMBAL_MANAGER_INFORMATION", RESPONSE_TIMEOUT)
    if msg is None:
        return

    cap = msg.cap_flags
    print(f"  time_boot_ms    : {msg.time_boot_ms} ms")
    print(f"  gimbal_device_id: {msg.gimbal_device_id}")
    print(f"  cap_flags (raw) : 0x{cap:08X}")
    print(f"  cap_flags       : {decode_flags(cap, MANAGER_CAP_FLAGS)}")
    print(f"  roll            : [{msg.roll_min*RAD2DEG:.1f}°, {msg.roll_max*RAD2DEG:.1f}°]")
    print(f"  pitch           : [{msg.pitch_min*RAD2DEG:.1f}°, {msg.pitch_max*RAD2DEG:.1f}°]")
    print(f"  yaw             : [{msg.yaw_min*RAD2DEG:.1f}°, {msg.yaw_max*RAD2DEG:.1f}°]")


# ── Stage 2 – GIMBAL_DEVICE_INFORMATION ──────────────────────────────────────

DEVICE_CAP_FLAGS = [
    (mavutil.mavlink.GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT,                  "HAS_RETRACT"),
    (mavutil.mavlink.GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL,                  "HAS_NEUTRAL"),
    (mavutil.mavlink.GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS,                "HAS_ROLL_AXIS"),
    (mavutil.mavlink.GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW,              "HAS_ROLL_FOLLOW"),
    (mavutil.mavlink.GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK,                "HAS_ROLL_LOCK"),
    (mavutil.mavlink.GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS,               "HAS_PITCH_AXIS"),
    (mavutil.mavlink.GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW,             "HAS_PITCH_FOLLOW"),
    (mavutil.mavlink.GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK,               "HAS_PITCH_LOCK"),
    (mavutil.mavlink.GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS,                 "HAS_YAW_AXIS"),
    (mavutil.mavlink.GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW,               "HAS_YAW_FOLLOW"),
    (mavutil.mavlink.GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK,                 "HAS_YAW_LOCK"),
    (mavutil.mavlink.GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW,        "SUPPORTS_INFINITE_YAW"),
    (mavutil.mavlink.GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_YAW_IN_EARTH_FRAME,  "SUPPORTS_YAW_IN_EARTH_FRAME"),
    (mavutil.mavlink.GIMBAL_DEVICE_CAP_FLAGS_HAS_RC_INPUTS,                "HAS_RC_INPUTS"),
]


def fetch_gimbal_device_information(mav: mavutil.mavfile) -> None:
    print(f"\n{'─'*60}")
    print(f"[2/3] GIMBAL_DEVICE_INFORMATION (msg_id={MSG_ID_GIMBAL_DEVICE_INFORMATION})")
    print(f"{'─'*60}")
    request_message(mav, MSG_ID_GIMBAL_DEVICE_INFORMATION)
    msg = wait_for_message(mav, "GIMBAL_DEVICE_INFORMATION", RESPONSE_TIMEOUT)
    if msg is None:
        return

    cap = msg.cap_flags
    has_earth_yaw = bool(cap & mavutil.mavlink.GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_YAW_IN_EARTH_FRAME)

    print(f"  vendor_name     : {bytes(msg.vendor_name).rstrip(b'\\x00').decode(errors='replace')}")
    print(f"  model_name      : {bytes(msg.model_name).rstrip(b'\\x00').decode(errors='replace')}")
    print(f"  custom_name     : {bytes(msg.custom_name).rstrip(b'\\x00').decode(errors='replace')}")
    print(f"  firmware_ver    : 0x{msg.firmware_version:08X}")
    print(f"  hardware_ver    : 0x{msg.hardware_version:08X}")
    print(f"  uid             : {msg.uid}")
    print(f"  cap_flags (raw) : 0x{cap:08X}")
    print(f"  cap_flags       : {decode_flags(cap, DEVICE_CAP_FLAGS)}")
    print(f"  roll            : [{msg.roll_min*RAD2DEG:.1f}°, {msg.roll_max*RAD2DEG:.1f}°]")
    print(f"  pitch           : [{msg.pitch_min*RAD2DEG:.1f}°, {msg.pitch_max*RAD2DEG:.1f}°]")
    print(f"  yaw             : [{msg.yaw_min*RAD2DEG:.1f}°, {msg.yaw_max*RAD2DEG:.1f}°]")

    if has_earth_yaw:
        print("\n  ✔  SUPPORTS_YAW_IN_EARTH_FRAME is SET")
        print("     → Gimbal hardware supports earth-frame yaw.")
        print("     → Ensure the autopilot is streaming AUTOPILOT_STATE_FOR_GIMBAL_DEVICE")
        print("       so that GIMBAL_DEVICE_FLAGS_CAN_ACCEPT_YAW_IN_EARTH_FRAME is raised at runtime.")
    else:
        print("\n  ✘  SUPPORTS_YAW_IN_EARTH_FRAME is NOT set")
        print("     Possible reasons:")
        print("       1. Gimbal hardware/firmware does not support earth-frame yaw.")
        print("       2. Autopilot is not streaming AUTOPILOT_STATE_FOR_GIMBAL_DEVICE —")
        print("          some gimbals only advertise this cap once they see that stream.")
        print("       3. Gimbal is connected standalone (no autopilot integration).")


# ── Stage 3 – GIMBAL_DEVICE_STATUS monitor ───────────────────────────────────

DEVICE_FLAGS = [
    (mavutil.mavlink.GIMBAL_DEVICE_FLAGS_RETRACT,                    "RETRACT"),
    (mavutil.mavlink.GIMBAL_DEVICE_FLAGS_NEUTRAL,                    "NEUTRAL"),
    (mavutil.mavlink.GIMBAL_DEVICE_FLAGS_ROLL_LOCK,                  "ROLL_LOCK"),
    (mavutil.mavlink.GIMBAL_DEVICE_FLAGS_PITCH_LOCK,                 "PITCH_LOCK"),
    (mavutil.mavlink.GIMBAL_DEVICE_FLAGS_YAW_LOCK,                   "YAW_LOCK"),
    (mavutil.mavlink.GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME,       "YAW_IN_VEHICLE_FRAME"),
    (mavutil.mavlink.GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME,         "YAW_IN_EARTH_FRAME"),
    (mavutil.mavlink.GIMBAL_DEVICE_FLAGS_ACCEPTS_YAW_IN_EARTH_FRAME, "ACCEPTS_YAW_IN_EARTH_FRAME"),
    (mavutil.mavlink.GIMBAL_DEVICE_FLAGS_RC_EXCLUSIVE,               "RC_EXCLUSIVE"),
    (mavutil.mavlink.GIMBAL_DEVICE_FLAGS_RC_MIXED,                   "RC_MIXED"),
]

CAN_ACCEPT_FLAG = mavutil.mavlink.GIMBAL_DEVICE_FLAGS_ACCEPTS_YAW_IN_EARTH_FRAME


def monitor_gimbal_device_status(mav: mavutil.mavfile, duration: float) -> None:
    print(f"\n{'─'*60}")
    print(f"[3/3] GIMBAL_DEVICE_STATUS monitor (msg_id={MSG_ID_GIMBAL_DEVICE_STATUS})")
    print(f"      Watching ACCEPTS_YAW_IN_EARTH_FRAME for {duration}s ...")
    print(f"      (Press Ctrl-C to stop early)")
    print(f"{'─'*60}")

    deadline   = time.time() + duration
    last_state = None   # track changes to avoid spamming identical lines

    try:
        while time.time() < deadline:
            msg = mav.recv_match(
                type="GIMBAL_DEVICE_STATUS",
                blocking=True,
                timeout=1.0,
            )
            if msg is None:
                continue

            flags     = msg.flags
            can_earth = bool(flags & CAN_ACCEPT_FLAG)
            state_str = "✔  CAN accept yaw-in-earth-frame" if can_earth else "✘  CANNOT accept yaw-in-earth-frame"

            if can_earth != last_state:
                # State changed — print a full detail block
                ts = f"{msg.time_boot_ms/1000:.3f}s"
                print(f"\n  [{ts}] *** STATE CHANGE ***  {state_str}")
                print(f"          flags (raw)   : 0x{flags:04X}")
                print(f"          flags         : {decode_flags(flags, DEVICE_FLAGS)}")
                q = msg.q  # attitude quaternion [w, x, y, z]
                print(f"          attitude q    : w={q[0]:.4f}  x={q[1]:.4f}  y={q[2]:.4f}  z={q[3]:.4f}")
                print(f"          failure_flags : 0x{msg.failure_flags:04X}")
                if not can_earth:
                    print("          ↳ Autopilot may not be streaming AUTOPILOT_STATE_FOR_GIMBAL_DEVICE.")
                last_state = can_earth
            else:
                # Same state — compact heartbeat line so user sees messages flowing
                ts = f"{msg.time_boot_ms/1000:.3f}s"
                icon = "✔" if can_earth else "✘"
                print(f"  [{ts}] {icon} earth-yaw={can_earth}  flags=0x{flags:04X}   ", end="\r")

    except KeyboardInterrupt:
        print("\n  [~] Monitor interrupted by user.")

    print(f"\n[*] Monitoring complete.")


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    mav = connect(CONNECTION_STRING)

    fetch_gimbal_manager_information(mav)
    fetch_gimbal_device_information(mav)

    if MONITOR:
        monitor_gimbal_device_status(mav, MONITOR_DURATION)
    else:
        print("\n[*] Skipping GIMBAL_DEVICE_STATUS monitor (--no-monitor)")

    mav.close()
    print("[*] Connection closed.")


if __name__ == "__main__":
    main()