#!/usr/bin/env python3
from __future__ import annotations
"""MANTA Flight Log Replay & Mission Planner Simulator."""

import sys
import os
import re
import time
import math
import argparse
import glob
try:
    import numpy as np
    import pandas as pd
    HAS_PANDAS_NUMPY = True
except ImportError:
    np = None
    pd = None
    HAS_PANDAS_NUMPY = False

try:
    from pymavlink import mavutil
    HAS_PYMAVLINK = True
    MAV_PARAM_TYPE_REAL32 = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    MAV_PARAM_TYPE_INT32 = mavutil.mavlink.MAV_PARAM_TYPE_INT32
except ImportError:
    mavutil = None
    HAS_PYMAVLINK = False
    MAV_PARAM_TYPE_REAL32 = 9
    MAV_PARAM_TYPE_INT32 = 6

# Path setup
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
GROUND_STATION_DIR = os.path.join(PROJECT_ROOT, "Code", "GROUND-STATION")
if GROUND_STATION_DIR not in sys.path:
    sys.path.insert(0, GROUND_STATION_DIR)

try:
    from MANTA_MISSION_PLANNER import ensure_mission_planner_autoconnect, launch_mission_planner
    HAS_MP_HELPERS = True
except ImportError:
    HAS_MP_HELPERS = False


def list_available_flight_logs(log_dir=None):
    """Finds all CSV flight logs recursively, prioritizing valid GPS and sorting chronologically (newest first)."""
    if log_dir is None:
        log_dir = os.path.join(GROUND_STATION_DIR, "flight_logs")

    logs = glob.glob(os.path.join(log_dir, "**", "*.csv"), recursive=True)

    def log_priority(path):
        try:
            df = pd.read_csv(path, nrows=50)
            has_gps = ("Latitude" in df.columns) and ((df["Latitude"] != 0.0).any())
            # Parse timestamp from filename like manta_flight_XXXX_YYYYMMDD_HHMMSS.csv
            fname = os.path.basename(path)
            match = re.search(r'(\d{8})_(\d{6})', fname)
            if match:
                ts = match.group(1) + match.group(2)
            else:
                ts = f"{int(os.path.getmtime(path)):014d}"
            return (1 if has_gps else 0, ts)
        except Exception:
            return (0, "0")

    logs.sort(key=log_priority, reverse=True)
    return logs


def calculate_battery_pct(voltage: float) -> int:
    """Calculates battery percentage for 3S/4S LiPo."""
    if voltage <= 0.0:
        return 0
    if voltage > 13.0:
        return int(max(0, min(100, (voltage - 13.6) / (16.8 - 13.6) * 100)))
    return int(max(0, min(100, (voltage - 10.2) / (12.6 - 10.2) * 100)))


def clean_flight_altitudes(df: pd.DataFrame) -> np.ndarray:
    """Filters out barometric sensor glitches (e.g. BMP280 1186m spikes)."""
    raw = df.get("Altitude (m)", df.get("Altitude", pd.Series([0.0] * len(df)))).values.astype(float)
    clean = raw.copy()
    for i in range(len(clean)):
        prev_a = clean[i - 1] if i > 0 else clean[0]
        if clean[i] > 120.0 or abs(clean[i] - prev_a) > 20.0:
            clean[i] = prev_a
    return clean


def prepare_flight_trajectory(df: pd.DataFrame, timestamps: np.ndarray):
    """Interpolates 1Hz GPS steps into a smooth 5Hz flight path with matching velocity vectors."""
    n = len(df)
    if n == 0:
        return [], [], [], [], [], []

    lats_raw = df.get("Latitude", pd.Series([0.0] * n)).values.astype(float)
    lons_raw = df.get("Longitude", pd.Series([0.0] * n)).values.astype(float)

    # 1. Identify distinct valid GPS fixes
    valid_mask = (lats_raw != 0.0) & (lons_raw != 0.0)
    if not np.any(valid_mask):
        zeros = [0.0] * n
        return zeros, zeros, zeros, zeros, zeros, zeros

    fix_indices = []
    for i in range(n):
        if valid_mask[i]:
            if len(fix_indices) == 0 or lats_raw[i] != lats_raw[fix_indices[-1]] or lons_raw[i] != lons_raw[fix_indices[-1]]:
                fix_indices.append(i)

    smooth_lats = np.zeros(n)
    smooth_lons = np.zeros(n)

    if len(fix_indices) == 1:
        smooth_lats[valid_mask] = lats_raw[fix_indices[0]]
        smooth_lons[valid_mask] = lons_raw[fix_indices[0]]
    else:
        fix_t = timestamps[fix_indices]
        fix_lat = lats_raw[fix_indices]
        fix_lon = lons_raw[fix_indices]

        # 2. Continuous time-based linear interpolation between distinct fixes
        smooth_lats = np.interp(timestamps, fix_t, fix_lat)
        smooth_lons = np.interp(timestamps, fix_t, fix_lon)

        # Extrapolate tail if aircraft was moving
        last_dt = float(fix_t[-1] - fix_t[-2])
        if last_dt > 0:
            v_n_last = (fix_lat[-1] - fix_lat[-2]) * 111139.0 / last_dt
            v_e_last = (fix_lon[-1] - fix_lon[-2]) * 111139.0 * math.cos(math.radians(fix_lat[-1])) / last_dt
            if math.hypot(v_n_last, v_e_last) > 1.0:
                for i in range(fix_indices[-1] + 1, n):
                    dt_extra = float(timestamps[i] - fix_t[-1])
                    smooth_lats[i] = fix_lat[-1] + (v_n_last / 111139.0) * dt_extra
                    smooth_lons[i] = fix_lon[-1] + (v_e_last / (111139.0 * math.cos(math.radians(fix_lat[-1])))) * dt_extra

        # Retain zero coordinates prior to first GPS fix
        for i in range(fix_indices[0]):
            if not valid_mask[i]:
                smooth_lats[i] = 0.0
                smooth_lons[i] = 0.0

    # 3. Derive ground velocity vectors (vx North, vy East, total speed)
    vx = [0.0] * n
    vy = [0.0] * n
    speeds = [0.0] * n
    cogs = [0.0] * n

    for i in range(1, n):
        if smooth_lats[i] == 0.0 and smooth_lons[i] == 0.0:
            continue
        dt = float(timestamps[i] - timestamps[i - 1])
        if dt <= 0:
            dt = 0.2
        d_north = (smooth_lats[i] - smooth_lats[i - 1]) * 111139.0
        d_east = (smooth_lons[i] - smooth_lons[i - 1]) * 111139.0 * math.cos(math.radians(smooth_lats[i]))
        v_north = d_north / dt
        v_east = d_east / dt
        spd = math.hypot(v_north, v_east)

        # Suppress ground jitter / stationary noise
        if spd < 0.5:
            vx[i] = 0.0
            vy[i] = 0.0
            speeds[i] = 0.0
            cogs[i] = cogs[i - 1]
        else:
            vx[i] = v_north
            vy[i] = v_east
            speeds[i] = spd
            cogs[i] = (math.degrees(math.atan2(v_east, v_north)) + 360.0) % 360.0

    vx[0] = vx[1] if n > 1 else 0.0
    vy[0] = vy[1] if n > 1 else 0.0
    speeds[0] = speeds[1] if n > 1 else 0.0
    cogs[0] = cogs[1] if n > 1 else 0.0

    return smooth_lats, smooth_lons, vx, vy, speeds, cogs


def estimate_flight_headings(df: pd.DataFrame, timestamps: np.ndarray, cogs: list, speeds: list) -> list:
    """Calculates flight heading by fusing GPS Course Over Ground with IMU Gyro Z."""
    n = len(df)
    if n == 0:
        return []

    # Gyro Z in deg/s
    if "Gyro Z (deg/s)" in df.columns:
        gz = df["Gyro Z (deg/s)"].values
    elif "Gyro Z (LSB)" in df.columns:
        gz = df["Gyro Z (LSB)"].values / 32.8
    else:
        gz = [0.0] * n

    # Initialize heading from first valid motion
    initial_heading = 0.0
    for i in range(n):
        if speeds[i] > 1.0:
            initial_heading = cogs[i]
            break

    fused_headings = [0.0] * n
    current_heading = initial_heading

    for i in range(n):
        dt = float(timestamps[i] - timestamps[i - 1]) if i > 0 else 0.2
        if dt <= 0 or dt > 1.0:
            dt = 0.2

        # Gyro rate integration with deadband filter
        rate = float(gz[i]) if abs(gz[i]) > 0.5 else 0.0
        current_heading = (current_heading + rate * dt) % 360.0

        # Complementary fusion with GPS COG during flight
        if speeds[i] > 1.0:
            diff = (cogs[i] - current_heading + 180.0) % 360.0 - 180.0
            alpha = min(0.35, max(0.08, 0.04 * speeds[i]))
            current_heading = (current_heading + alpha * diff) % 360.0

        fused_headings[i] = current_heading

    return fused_headings


REPLAY_PARAMETERS = [
    (b"SYSID_SW_MREV", 120.0, MAV_PARAM_TYPE_REAL32),
    (b"STAT_RUNTIME", 1.0, MAV_PARAM_TYPE_REAL32),
    (b"STAT_FLTTIME", 1.0, MAV_PARAM_TYPE_REAL32),
    (b"SYSID_THISMAV", 1.0, MAV_PARAM_TYPE_INT32),
    (b"FRAME_CLASS", 1.0, MAV_PARAM_TYPE_INT32),
    (b"ARMING_CHECK", 0.0, MAV_PARAM_TYPE_INT32),
]


def handle_mission_planner_requests(mav_conn, home_lat_e7=0, home_lon_e7=0, home_alt_mm=0, cur_lat_e7=0, cur_lon_e7=0, cur_alt_mm=0):
    """Processes incoming requests from Mission Planner non-blockingly.

    Immediately satisfies Mission Planner so parameter popups close and Home Location
    updates/queries succeed without 'Failed to update home location' warnings.
    """
    if not mav_conn:
        return home_lat_e7, home_lon_e7, home_alt_mm

    try:
        while True:
            try:
                msg = mav_conn.recv_msg()
            except (ConnectionResetError, OSError):
                break
            if not msg:
                break

            msg_type = msg.get_type()
            src_sys = msg.get_srcSystem()
            src_comp = msg.get_srcComponent()

            if msg_type == "PARAM_REQUEST_LIST":
                total = len(REPLAY_PARAMETERS)
                for idx, (p_id, p_val, p_type) in enumerate(REPLAY_PARAMETERS):
                    mav_conn.mav.param_value_send(p_id, p_val, p_type, total, idx)

            elif msg_type == "PARAM_REQUEST_READ":
                req_id = getattr(msg, 'param_id', b"")
                if isinstance(req_id, bytes):
                    clean_id = req_id.split(b"\x00")[0].decode("ascii", errors="ignore")
                else:
                    clean_id = str(req_id)
                matched = False
                total = len(REPLAY_PARAMETERS)
                for idx, (p_id, p_val, p_type) in enumerate(REPLAY_PARAMETERS):
                    if p_id.decode("ascii", errors="ignore").startswith(clean_id):
                        mav_conn.mav.param_value_send(p_id, p_val, p_type, total, idx)
                        matched = True
                        break
                if not matched:
                    mav_conn.mav.param_value_send(
                        req_id if isinstance(req_id, bytes) else req_id.encode("ascii")[:16],
                        0.0,
                        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
                        total,
                        0
                    )

            # 1. Mission / Waypoint Protocol (Mission Planner treats Waypoint 0 as Home)
            elif msg_type == "MISSION_REQUEST_LIST":
                has_home = (home_lat_e7 != 0 or home_lon_e7 != 0)
                mav_conn.mav.mission_count_send(src_sys, src_comp, 1 if has_home else 0)

            elif msg_type in ("MISSION_REQUEST", "MISSION_REQUEST_INT"):
                req_seq = getattr(msg, 'seq', 0)
                if req_seq == 0 and (home_lat_e7 != 0 or home_lon_e7 != 0):
                    if msg_type == "MISSION_REQUEST":
                        mav_conn.mav.mission_item_send(
                            src_sys, src_comp, 0,
                            mavutil.mavlink.MAV_FRAME_GLOBAL,
                            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                            0, 1, 0.0, 0.0, 0.0, 0.0,
                            float(home_lat_e7) / 1e7, float(home_lon_e7) / 1e7, float(home_alt_mm) / 1000.0
                        )
                    else:
                        mav_conn.mav.mission_item_int_send(
                            src_sys, src_comp, 0,
                            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                            0, 1, 0.0, 0.0, 0.0, 0.0,
                            home_lat_e7, home_lon_e7, float(home_alt_mm) / 1000.0
                        )
                else:
                    mav_conn.mav.mission_ack_send(src_sys, src_comp, mavutil.mavlink.MAV_MISSION_ACCEPTED)

            elif msg_type in ("MISSION_COUNT", "MISSION_WRITE_PARTIAL_LIST"):
                # Mission Planner is writing Home waypoint (seq 0)
                mav_conn.mav.mission_request_int_send(src_sys, src_comp, 0)

            elif msg_type in ("MISSION_ITEM", "MISSION_ITEM_INT"):
                seq = getattr(msg, 'seq', 0)
                if seq == 0:
                    item_x = getattr(msg, 'x', 0)
                    item_y = getattr(msg, 'y', 0)
                    item_z = getattr(msg, 'z', 0.0)
                    if msg_type == "MISSION_ITEM":
                        if item_x != 0 or item_y != 0:
                            home_lat_e7 = int(round(item_x * 1e7))
                            home_lon_e7 = int(round(item_y * 1e7))
                            home_alt_mm = int(round(item_z * 1000))
                    else:
                        if item_x != 0 or item_y != 0:
                            home_lat_e7 = int(item_x)
                            home_lon_e7 = int(item_y)
                            home_alt_mm = int(round(item_z * 1000))
                mav_conn.mav.mission_ack_send(src_sys, src_comp, mavutil.mavlink.MAV_MISSION_ACCEPTED)
                if home_lat_e7 != 0 or home_lon_e7 != 0:
                    try:
                        mav_conn.mav.home_position_send(
                            home_lat_e7, home_lon_e7, home_alt_mm,
                            0.0, 0.0, 0.0, [1.0, 0.0, 0.0, 0.0], 0.0, 0.0, 0.0
                        )
                    except Exception:
                        pass

            elif msg_type == "MISSION_ACK":
                pass

            elif msg_type == "MISSION_SET_CURRENT":
                mav_conn.mav.mission_current_send(0)

            # 2. Commands (COMMAND_LONG and COMMAND_INT for DO_SET_HOME & GET_HOME)
            elif msg_type in ("COMMAND_LONG", "COMMAND_INT"):
                cmd = getattr(msg, 'command', 0)

                # MAV_CMD_DO_SET_HOME (179)
                if cmd in (179, getattr(mavutil.mavlink, 'MAV_CMD_DO_SET_HOME', 179)):
                    p1 = getattr(msg, 'param1', 1.0)
                    if p1 == 1.0 and (cur_lat_e7 != 0 or cur_lon_e7 != 0):
                        home_lat_e7 = cur_lat_e7
                        home_lon_e7 = cur_lon_e7
                        home_alt_mm = cur_alt_mm
                    elif msg_type == "COMMAND_INT":
                        c_x = getattr(msg, 'x', 0)
                        c_y = getattr(msg, 'y', 0)
                        c_z = getattr(msg, 'z', 0.0)
                        if c_x != 0 or c_y != 0:
                            home_lat_e7 = int(c_x)
                            home_lon_e7 = int(c_y)
                            home_alt_mm = int(round(c_z * 1000))
                    else:
                        p5 = getattr(msg, 'param5', 0.0)
                        p6 = getattr(msg, 'param6', 0.0)
                        p7 = getattr(msg, 'param7', 0.0)
                        if p5 != 0.0 or p6 != 0.0:
                            home_lat_e7 = int(round(p5 * 1e7))
                            home_lon_e7 = int(round(p6 * 1e7))
                            home_alt_mm = int(round(p7 * 1000))

                    try:
                        mav_conn.mav.command_ack_send(cmd, mavutil.mavlink.MAV_RESULT_ACCEPTED)
                        if home_lat_e7 != 0 or home_lon_e7 != 0:
                            mav_conn.mav.home_position_send(
                                home_lat_e7, home_lon_e7, home_alt_mm,
                                0.0, 0.0, 0.0, [1.0, 0.0, 0.0, 0.0], 0.0, 0.0, 0.0
                            )
                    except Exception:
                        pass

                # MAV_CMD_GET_HOME_POSITION (410)
                elif cmd in (410, getattr(mavutil.mavlink, 'MAV_CMD_GET_HOME_POSITION', 410)):
                    try:
                        mav_conn.mav.command_ack_send(cmd, mavutil.mavlink.MAV_RESULT_ACCEPTED)
                        if home_lat_e7 != 0 or home_lon_e7 != 0:
                            mav_conn.mav.home_position_send(
                                home_lat_e7, home_lon_e7, home_alt_mm,
                                0.0, 0.0, 0.0, [1.0, 0.0, 0.0, 0.0], 0.0, 0.0, 0.0
                            )
                    except Exception:
                        pass

                else:
                    try:
                        mav_conn.mav.command_ack_send(cmd, mavutil.mavlink.MAV_RESULT_ACCEPTED)
                    except Exception:
                        pass
    except Exception:
        pass

    return home_lat_e7, home_lon_e7, home_alt_mm


def send_mavlink_frame(
    mav_conn,
    time_boot_ms: int,
    now: float,
    lat: float,
    lon: float,
    alt_m: float,
    roll: float,
    pitch: float,
    yaw: float,
    airspeed: float,
    gnd_spd: float,
    vx_mps: float,
    vy_mps: float,
    climb_rate: float,
    throttle_pct: int,
    battery_v: float,
    satellites: int,
    fix_type: int,
    rc_lost: bool
):
    """Transmits all standard MAVLink telemetry packets for a single state."""
    time_usec = int(now * 1e6) & 0xFFFFFFFFFFFFFFFF
    alt_mm = int(alt_m * 1000)
    batt_mv = int(max(0.0, battery_v) * 1000)
    batt_pct = calculate_battery_pct(battery_v)

    # 1. HEARTBEAT (Generic autopilot avoids parameter requests hanging in MP)
    state = mavutil.mavlink.MAV_STATE_EMERGENCY if rc_lost else mavutil.mavlink.MAV_STATE_ACTIVE
    mav_conn.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_FIXED_WING,
        mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
        mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED | mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED,
        0,
        state
    )

    # 2. EXTENDED_SYS_STATE (In-air detection)
    mav_conn.mav.extended_sys_state_send(
        mavutil.mavlink.MAV_VTOL_STATE_UNDEFINED,
        mavutil.mavlink.MAV_LANDED_STATE_IN_AIR if (throttle_pct > 0 or alt_m > 1.0) else mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
    )

    # 3. SYS_STATUS & BATTERY_STATUS
    lat_e7 = int(round(lat * 1e7))
    lon_e7 = int(round(lon * 1e7))
    has_pos = (lat_e7 != 0 or lon_e7 != 0)

    mask = (
        mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO |
        mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
        mavutil.mavlink.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
        mavutil.mavlink.MAV_SYS_STATUS_SENSOR_BATTERY
    )
    if has_pos:
        mask |= mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS

    mav_conn.mav.sys_status_send(mask, mask, mask, 500, batt_mv, -1, batt_pct, 0, 0, 0, 0, 0, 0)
    mav_conn.mav.battery_status_send(
        0, mavutil.mavlink.MAV_BATTERY_FUNCTION_ALL, mavutil.mavlink.MAV_BATTERY_TYPE_LIPO,
        2500, [batt_mv] + [65535] * 9, -1, -1, -1, batt_pct
    )

    # 4. ATTITUDE
    mav_conn.mav.attitude_send(time_boot_ms, math.radians(roll), math.radians(pitch), math.radians(yaw), 0.0, 0.0, 0.0)

    # 5. VFR_HUD
    mav_conn.mav.vfr_hud_send(float(airspeed), float(gnd_spd), int(yaw % 360), throttle_pct, alt_m, float(climb_rate))

    # 6. GLOBAL_POSITION_INT
    vz = int(max(-32000, min(32000, -climb_rate * 100)))
    vx = int(max(-32000, min(32000, round(vx_mps * 100)))) if has_pos else 0
    vy = int(max(-32000, min(32000, round(vy_mps * 100)))) if has_pos else 0
    hdg_cdeg = int((yaw % 360) * 100) & 0xFFFF

    mav_conn.mav.global_position_int_send(
        time_boot_ms,
        lat_e7 if has_pos else 0,
        lon_e7 if has_pos else 0,
        alt_mm, alt_mm, vx, vy, vz,
        hdg_cdeg
    )

    # 7. GPS_RAW_INT (HDOP eph=100 in cm / 1.0m, VDOP epv=150 in cm / 1.5m to eliminate 'Bad GPS Pos' alarm)
    raw_lat = lat_e7 if has_pos else 2147483647
    raw_lon = lon_e7 if has_pos else 2147483647
    gps_fix = max(3, fix_type) if (has_pos and satellites >= 4) else (fix_type if has_pos else 0)
    eph = 100 if has_pos else 65535  # 1.00 m HDOP
    epv = 150 if has_pos else 65535  # 1.50 m VDOP
    sats = max(12, satellites) if has_pos else satellites
    mav_conn.mav.gps_raw_int_send(
        time_usec, gps_fix, raw_lat, raw_lon, alt_mm,
        eph, epv, int(gnd_spd * 100), hdg_cdeg, sats
    )


def replay_flight_log(
    csv_path: str,
    speed_factor: float = 1.0,
    udp_target: str = "udpout:127.0.0.1:14550",
    launch_mp: bool = False,
    loop: bool = False,
    fast_forward_gaps: bool = True,
    mp_delay: float = 5.0
):
    """Streams CSV telemetry as MAVLink packets over UDP to Mission Planner."""
    if not os.path.exists(csv_path):
        print(f"[Error] Log not found: {csv_path}")
        return

    if not HAS_PYMAVLINK:
        print("[Error] 'pymavlink' is not installed.")
        return

    # Auto-launch Mission Planner
    if launch_mp and HAS_MP_HELPERS:
        launch_mission_planner()

    print(f"\n[Replay] File: {os.path.basename(csv_path)} | Speed: {speed_factor:.1f}x | UDP: {udp_target}")

    try:
        df = pd.read_csv(csv_path)
    except Exception as e:
        print(f"[Error] Failed to read CSV: {e}")
        return

    if len(df) == 0:
        print("[Error] CSV file is empty.")
        return

    # Extract timestamps and guarantee strict chronological order
    time_col = None
    for col in ["Elapsed Time (s)", "ESP32 Timestamp (ms)", "Timestamp (ms)"]:
        if col in df.columns:
            time_col = col
            break

    if time_col:
        df = df.sort_values(by=time_col).reset_index(drop=True)

    if time_col == "Elapsed Time (s)":
        timestamps = df[time_col].values.astype(float)
    elif time_col:
        timestamps = (df[time_col].values / 1000.0).astype(float)
    else:
        timestamps = np.array([i * 0.2 for i in range(len(df))])

    flight_start_t = timestamps[0] if len(timestamps) > 0 else 0.0

    # Pre-process smooth GPS trajectory & filter altitude sensor glitches
    smooth_lats, smooth_lons, vx_list, vy_list, speeds, cogs = prepare_flight_trajectory(df, timestamps)
    clean_alts = clean_flight_altitudes(df)
    headings = estimate_flight_headings(df, timestamps, cogs, speeds)

    # Build continuous playback timeline
    playback_times = [0.0]
    for i in range(1, len(timestamps)):
        dt = float(timestamps[i] - timestamps[i - 1])
        if fast_forward_gaps and dt > 1.5:
            # Measure actual geographic displacement across gap
            d_n = (smooth_lats[i] - smooth_lats[i - 1]) * 111139.0
            d_e = (smooth_lons[i] - smooth_lons[i - 1]) * 111139.0 * math.cos(math.radians(smooth_lats[i - 1]))
            dist = math.hypot(d_n, d_e)
            dt_step = min(max(2.0, dist / 20.0), 6.0) if dist > 5.0 else 1.0
        else:
            dt_step = max(0.05, dt)
        playback_times.append(playback_times[-1] + dt_step)

    # Pre-extract Home location from first valid GPS record in dataset
    valid_gps_mask = (df.get("Latitude", pd.Series([0.0] * len(df))) != 0.0) & (df.get("Longitude", pd.Series([0.0] * len(df))) != 0.0)
    if valid_gps_mask.any():
        first_gps_idx = valid_gps_mask.idxmax()
        home_lat_e7 = int(round(float(df["Latitude"].iloc[first_gps_idx]) * 1e7))
        home_lon_e7 = int(round(float(df["Longitude"].iloc[first_gps_idx]) * 1e7))
        home_alt_mm = int(round(float(clean_alts[first_gps_idx]) * 1000))
    else:
        home_lat_e7 = home_lon_e7 = home_alt_mm = 0

    # Init MAVLink connection
    try:
        mav_conn = mavutil.mavlink_connection(udp_target, source_system=1, source_component=1)
    except Exception as e:
        print(f"[Error] MAVLink init failed: {e}")
        return

    # If Mission Planner was launched, wait for it to open and auto-connect UDP
    if launch_mp and mp_delay > 0:
        print(f"[Mission Planner] Waiting {int(mp_delay)}s for Mission Planner to open and connect UDP...")
        start_wait = time.time()
        last_sec = int(mp_delay)
        last_hb_time = 0.0
        while time.time() - start_wait < mp_delay:
            remaining = int(math.ceil(mp_delay - (time.time() - start_wait)))
            if remaining != last_sec:
                last_sec = remaining
                sys.stdout.write(f"\r[Mission Planner] Starting replay stream in {remaining}s... ")
                sys.stdout.flush()

            now_t = time.time()
            if mav_conn and (now_t - last_hb_time) >= 1.0:
                last_hb_time = now_t
                mav_conn.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_FIXED_WING,
                    mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                    mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED | mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED,
                    0,
                    mavutil.mavlink.MAV_STATE_ACTIVE
                )
                if home_lat_e7 != 0 or home_lon_e7 != 0:
                    try:
                        mav_conn.mav.home_position_send(
                            home_lat_e7, home_lon_e7, home_alt_mm,
                            0.0, 0.0, 0.0, [1.0, 0.0, 0.0, 0.0], 0.0, 0.0, 0.0
                        )
                        mav_conn.mav.gps_global_origin_send(home_lat_e7, home_lon_e7, home_alt_mm)
                    except Exception:
                        pass

            home_lat_e7, home_lon_e7, home_alt_mm = handle_mission_planner_requests(mav_conn, home_lat_e7, home_lon_e7, home_alt_mm)
            time.sleep(0.05)

        sys.stdout.write("\r[Mission Planner] Ready! Starting telemetry playback...          \n")
        sys.stdout.flush()

    print("[Simulator] Telemetry active. Streaming to Mission Planner (Ctrl+C to stop).")

    while True:
        sim_start = time.time()
        home_set = False
        last_alt_calc = last_alt_time = filtered_climb = 0.0
        first_alt = True
        total_rows = len(df)

        for i in range(total_rows):
            row = df.iloc[i]
            target_t = playback_times[i] / max(0.01, speed_factor)

            # Bridge large signal dropouts smoothly (prevent instant teleportation)
            dt_raw = float(timestamps[i] - timestamps[i - 1]) if i > 0 else 0.2
            if i > 0 and dt_raw > 1.5:
                lat1, lon1, alt1 = smooth_lats[i - 1], smooth_lons[i - 1], clean_alts[i - 1]
                lat2, lon2, alt2 = smooth_lats[i], smooth_lons[i], clean_alts[i]
                d_north = (lat2 - lat1) * 111139.0
                d_east = (lon2 - lon1) * 111139.0 * math.cos(math.radians(lat1))
                dist = math.hypot(d_north, d_east)

                bridge_dur = (playback_times[i] - playback_times[i - 1]) / max(0.01, speed_factor)
                k_steps = max(1, int(bridge_dur / 0.2))
                bridge_vx = d_north / max(0.1, bridge_dur)
                bridge_vy = d_east / max(0.1, bridge_dur)
                bridge_spd = dist / max(0.1, bridge_dur)
                bridge_climb = (alt2 - alt1) / max(0.1, bridge_dur)
                bridge_bearing = (math.degrees(math.atan2(d_east, d_north)) + 360.0) % 360.0 if dist > 2.0 else headings[i - 1]

                for k in range(1, k_steps):
                    alpha = k / k_steps
                    b_lat = (1 - alpha) * lat1 + alpha * lat2
                    b_lon = (1 - alpha) * lon1 + alpha * lon2
                    b_alt = (1 - alpha) * alt1 + alpha * alt2
                    b_pitch = (1 - alpha) * float(df.iloc[i - 1].get("Pitch (deg)", 0.0)) + alpha * float(row.get("Pitch (deg)", 0.0))
                    b_roll = (1 - alpha) * float(df.iloc[i - 1].get("Roll (deg)", 0.0)) + alpha * float(row.get("Roll (deg)", 0.0))
                    b_yaw = bridge_bearing

                    b_now = time.time()
                    b_boot_ms = int((b_now - sim_start) * 1000) & 0xFFFFFFFF
                    send_mavlink_frame(
                        mav_conn, b_boot_ms, b_now, b_lat, b_lon, b_alt, b_roll, b_pitch, b_yaw,
                        airspeed=max(bridge_spd, 12.0), gnd_spd=bridge_spd,
                        vx_mps=bridge_vx, vy_mps=bridge_vy, climb_rate=bridge_climb,
                        throttle_pct=int(max(0, min(100, (float(row.get("ESC Throttle (us)", 1000)) - 1000) / 10))),
                        battery_v=float(row.get("Battery Voltage (V)", 15.0)),
                        satellites=int(row.get("Satellites", 0)),
                        fix_type=int(row.get("Fix Type", 3 if (b_lat != 0 or b_lon != 0) else 0)),
                        rc_lost=True
                    )
                    home_lat_e7, home_lon_e7, home_alt_mm = handle_mission_planner_requests(
                        mav_conn, home_lat_e7, home_lon_e7, home_alt_mm,
                        int(round(b_lat * 1e7)), int(round(b_lon * 1e7)), int(round(b_alt * 1000))
                    )
                    if k % 10 == 0 and (home_lat_e7 != 0 or home_lon_e7 != 0):
                        try:
                            mav_conn.mav.home_position_send(
                                home_lat_e7, home_lon_e7, home_alt_mm,
                                0.0, 0.0, 0.0, [1.0, 0.0, 0.0, 0.0], 0.0, 0.0, 0.0
                            )
                        except Exception:
                            pass

                    sys.stdout.write(f"\r[LINK DROPOUT {dt_raw:4.1f}s] Bridging to next GPS fix ({dist:4.0f}m)...   ")
                    sys.stdout.flush()
                    time.sleep(0.2 / max(0.01, speed_factor))

            # Timing synchronization
            sleep_needed = target_t - (time.time() - sim_start)
            while sleep_needed > 0.5:
                time.sleep(0.5)
                mav_conn.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_FIXED_WING,
                    mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                    mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED | mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED,
                    0,
                    mavutil.mavlink.MAV_STATE_ACTIVE
                )
                home_lat_e7, home_lon_e7, home_alt_mm = handle_mission_planner_requests(mav_conn, home_lat_e7, home_lon_e7, home_alt_mm)
                sleep_needed = target_t - (time.time() - sim_start)
            if sleep_needed > 0:
                time.sleep(sleep_needed)

            now = time.time()
            time_boot_ms = int((now - sim_start) * 1000) & 0xFFFFFFFF

            pitch = float(row.get("Pitch (deg)", 0.0))
            roll = float(row.get("Roll (deg)", 0.0))
            yaw = float(headings[i]) if i < len(headings) else float(row.get("Yaw (deg)", 0.0))
            alt_m = float(clean_alts[i])

            lat = float(smooth_lats[i])
            lon = float(smooth_lons[i])
            gnd_spd = float(speeds[i])
            vx_mps = float(vx_list[i])
            vy_mps = float(vy_list[i])

            satellites = int(row.get("Satellites", 0))
            fix_type = int(row.get("Fix Type", 3 if (lat != 0 or lon != 0) else 0))

            battery_v = float(row.get("Battery Voltage (V)", 15.0))
            esc_us = float(row.get("ESC Throttle (us)", row.get("RC3 Throttle (us)", 1000)))
            throttle_pct = int(max(0, min(100, (esc_us - 1000) / 10)))
            rc_lost = bool(row.get("RC Signal Lost", 0))
            flight_mode = int(row.get("Flight Mode", 1))

            airspeed = max(gnd_spd, (throttle_pct / 100.0) * 18.0)
            if first_alt:
                last_alt_calc = alt_m
                last_alt_time = now
                first_alt = False
            else:
                dt_alt = now - last_alt_time
                if dt_alt >= 0.05:
                    raw_climb = (alt_m - last_alt_calc) / dt_alt
                    if abs(raw_climb) < 30.0:
                        filtered_climb = 0.7 * filtered_climb + 0.3 * raw_climb
                    last_alt_calc = alt_m
                    last_alt_time = now

            # Broadcast complete telemetry frame
            send_mavlink_frame(
                mav_conn, time_boot_ms, now, lat, lon, alt_m, roll, pitch, yaw,
                airspeed, gnd_spd, vx_mps, vy_mps, filtered_climb,
                throttle_pct, battery_v, satellites, fix_type, rc_lost
            )

            # Maintain Home Position and GPS Origin continuously in Mission Planner
            if (home_lat_e7 != 0 or home_lon_e7 != 0) and not home_set:
                home_set = True
                try:
                    mav_conn.mav.home_position_send(
                        home_lat_e7, home_lon_e7, home_alt_mm,
                        0.0, 0.0, 0.0, [1.0, 0.0, 0.0, 0.0], 0.0, 0.0, 0.0
                    )
                    mav_conn.mav.gps_global_origin_send(home_lat_e7, home_lon_e7, home_alt_mm)
                except Exception:
                    pass
            elif (home_lat_e7 != 0 or home_lon_e7 != 0) and i % 10 == 0:
                try:
                    mav_conn.mav.home_position_send(
                        home_lat_e7, home_lon_e7, home_alt_mm,
                        0.0, 0.0, 0.0, [1.0, 0.0, 0.0, 0.0], 0.0, 0.0, 0.0
                    )
                except Exception:
                    pass

            # Non-blocking service for Mission Planner queries (parameters, missions, commands)
            home_lat_e7, home_lon_e7, home_alt_mm = handle_mission_planner_requests(
                mav_conn, home_lat_e7, home_lon_e7, home_alt_mm,
                int(round(lat * 1e7)), int(round(lon * 1e7)), int(round(alt_m * 1000))
            )

            # Progress feedback
            if i % 10 == 0 or i == total_rows - 1:
                prog = (i + 1) / total_rows * 100
                rel_flight_t = float(timestamps[i] - flight_start_t)
                sys.stdout.write(
                    f"\r[{prog:5.1f}%] Sim T: {target_t:5.1f}s | Log T: {rel_flight_t:5.1f}s | HDG: {yaw:5.1f}° | M{flight_mode} | "
                    f"Alt: {alt_m:4.1f}m | Spd: {gnd_spd:4.1f}m/s | Thr: {throttle_pct:2d}% | Sats: {satellites:2d}   "
                )
                sys.stdout.flush()

        print(f"\n[Replay Complete] Finished streaming {total_rows} records.")
        if not loop:
            break
        print("[Replay] Looping...\n")
        time.sleep(1.0)


def interactive_selector():
    """CLI prompt to choose a flight log with descriptive metadata."""
    logs = list_available_flight_logs()
    if not logs:
        print("[Error] No logs found.")
        sys.exit(1)

    if not sys.stdin.isatty():
        return logs[0]

    print("\nMANTA Flight Logs:")
    for idx, log in enumerate(logs):
        fname = os.path.basename(log)
        try:
            df_hdr = pd.read_csv(log)
            rows = len(df_hdr)
            has_gps = ("Latitude" in df_hdr.columns) and ((df_hdr["Latitude"] != 0.0).any())
            if has_gps and rows > 1000:
                tag = "FULL FLIGHT (GPS 3D Fix)"
            elif has_gps:
                tag = "RADIO GAPS (GPS 3D Fix)"
            else:
                tag = "LEGACY / NO GPS"
            dur = (df_hdr["Elapsed Time (s)"].iloc[-1] - df_hdr["Elapsed Time (s)"].iloc[0]) if "Elapsed Time (s)" in df_hdr.columns else 0.0
            print(f" [{idx + 1}] {fname:42s} | {rows:4d} pts ({dur:4.0f}s) | [{tag}]")
        except Exception:
            print(f" [{idx + 1}] {fname}")

    try:
        choice = input(f"\nSelect log [1-{len(logs)}] (Default: 1): ").strip()
        if not choice:
            return logs[0]
        idx = int(choice) - 1
        return logs[idx if 0 <= idx < len(logs) else 0]
    except (ValueError, EOFError):
        return logs[0]


def main():
    parser = argparse.ArgumentParser(description="MANTA Flight Replayer")
    parser.add_argument("--log", type=str, default=None, help="CSV log path.")
    parser.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier.")
    parser.add_argument("--udp", type=str, default="udpout:127.0.0.1:14550", help="MAVLink UDP destination.")
    parser.add_argument("--no-mp", action="store_true", help="Do not automatically launch Mission Planner.")
    parser.add_argument("--launch-mp", action="store_true", help="Launch Mission Planner (default: True).")
    parser.add_argument("--loop", action="store_true", help="Loop playback.")
    parser.add_argument("--keep-gaps", action="store_true", help="Wait full real-time duration of signal loss gaps.")
    parser.add_argument("--mp-delay", type=float, default=5.0, help="Seconds to wait for Mission Planner to open before streaming (default: 5.0s).")

    args = parser.parse_args()
    log = args.log or interactive_selector()
    should_launch_mp = not args.no_mp

    try:
        replay_flight_log(
            log,
            speed_factor=args.speed,
            udp_target=args.udp,
            launch_mp=should_launch_mp,
            loop=args.loop,
            fast_forward_gaps=not args.keep_gaps,
            mp_delay=args.mp_delay if should_launch_mp else 0.0
        )
    except KeyboardInterrupt:
        print("\n[Simulator] Stopped.")


if __name__ == "__main__":
    main()
