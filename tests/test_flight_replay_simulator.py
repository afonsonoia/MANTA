"""Unit tests for the MANTA Flight Log Replayer & Simulator."""

import os
import sys
import pytest
import threading
import time

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
GROUND_STATION_DIR = os.path.join(PROJECT_ROOT, "Code", "GROUND-STATION")
if GROUND_STATION_DIR not in sys.path:
    sys.path.insert(0, GROUND_STATION_DIR)

try:
    from pymavlink import mavutil
    HAS_PYMAVLINK = True
except ImportError:
    HAS_PYMAVLINK = False

try:
    import pandas as pd
    import numpy as np
    HAS_PANDAS_NUMPY = True
except ImportError:
    HAS_PANDAS_NUMPY = False

from replay_flight_mission_planner import (
    list_available_flight_logs,
    calculate_battery_pct,
    clean_flight_altitudes,
    prepare_flight_trajectory,
    estimate_flight_headings,
    replay_flight_log,
)


def test_list_flight_logs(tmp_path):
    logs = list_available_flight_logs()
    if len(logs) == 0:
        # In CI where flight_logs is gitignored, test with a mock log in tmp_path
        mock_log = tmp_path / "manta_flight_0001_20260901_100000.csv"
        mock_log.write_text("Record Number,Elapsed Time (s),Latitude,Longitude\n1,0.0,32.7,-16.8\n")
        logs = list_available_flight_logs(log_dir=str(tmp_path))
    assert len(logs) > 0
    for l in logs:
        assert os.path.exists(l)


def test_battery_percentage_calculation():
    assert calculate_battery_pct(16.8) == 100
    assert calculate_battery_pct(15.2) in (49, 50)
    assert calculate_battery_pct(13.6) == 0
    assert calculate_battery_pct(0.0) == 0


@pytest.mark.skipif(not HAS_PYMAVLINK or not HAS_PANDAS_NUMPY, reason="pymavlink, pandas, or numpy not installed")
def test_replay_playback_network_stream(tmp_path):
    """Validates MAVLink UDP streaming to listener."""
    logs = list_available_flight_logs()
    if len(logs) == 0:
        sample_log = str(tmp_path / "manta_mock_flight.csv")
        with open(sample_log, "w", encoding="utf-8") as f:
            f.write("Record Number,Elapsed Time (s),Roll (deg),Pitch (deg),Yaw (deg),Latitude,Longitude,Altitude (m),Battery Voltage (V),Throttle (PWM)\n")
            for i in range(15):
                f.write(f"{i},{i*0.1},0.0,2.0,90.0,32.7259,-16.8850,50.0,15.2,1500\n")
    else:
        sample_log = logs[0]

    t = threading.Thread(
        target=replay_flight_log,
        kwargs={
            "csv_path": sample_log,
            "speed_factor": 10.0,
            "udp_target": "udpout:127.0.0.1:14552",
            "launch_mp": False,
            "loop": False
        },
        daemon=True
    )

    mav_in = mavutil.mavlink_connection("udpin:127.0.0.1:14552")
    t.start()

    received_types = set()
    start_wait = time.time()
    while time.time() - start_wait < 3.0:
        msg = mav_in.recv_msg()
        if msg:
            received_types.add(msg.get_type())
        else:
            time.sleep(0.02)

    assert "HEARTBEAT" in received_types
    assert "ATTITUDE" in received_types
    assert "GLOBAL_POSITION_INT" in received_types


@pytest.mark.skipif(not HAS_PANDAS_NUMPY, reason="pandas or numpy not installed")
def test_trajectory_smoothing_and_velocity():
    """Tests 1Hz to 5Hz interpolation and velocity vector derivation."""
    # 10 points representing 2 seconds at 5Hz, with GPS updating every 1s (rows 0-4 same, 5-9 same)
    # Moving due North by 11.11 meters (~0.0001 deg lat in 1 sec => ~11.1 m/s)
    timestamps = np.array([i * 0.2 for i in range(10)])
    lats = [41.0] * 5 + [41.0001] * 5
    lons = [-8.0] * 10
    df = pd.DataFrame({"Latitude": lats, "Longitude": lons})

    s_lats, s_lons, vx, vy, spd, cog = prepare_flight_trajectory(df, timestamps)

    assert len(s_lats) == 10
    # Between index 0 and index 5, intermediate points should strictly increase smoothly
    assert s_lats[0] < s_lats[2] < s_lats[5]
    # Speeds should be consistent around ~11 m/s rather than alternating between 0 and 55 m/s
    for i in range(1, 10):
        assert 10.0 <= spd[i] <= 13.0
        assert 10.0 <= vx[i] <= 13.0
        assert abs(vy[i]) < 1.0


@pytest.mark.skipif(not HAS_PANDAS_NUMPY, reason="pandas or numpy not installed")
def test_heading_estimation_fusion():
    """Tests heading fusion combining Gyro Z and GPS COG."""
    n = 20
    timestamps = np.array([i * 0.2 for i in range(n)])
    speeds = [12.0] * n
    cogs = [90.0] * n  # Heading East
    df = pd.DataFrame({
        "Gyro Z (deg/s)": [0.0] * n
    })

    headings = estimate_flight_headings(df, timestamps, cogs, speeds)
    assert len(headings) == n
    assert abs(headings[-1] - 90.0) < 1.0


@pytest.mark.skipif(not HAS_PANDAS_NUMPY, reason="pandas or numpy not installed")
def test_clean_flight_altitudes():
    """Tests rejection of extreme sensor spikes (e.g. 1186m)."""
    df = pd.DataFrame({
        "Altitude (m)": [10.0, 10.5, 1186.8, 1186.8, 11.0, 11.2]
    })
    cleaned = clean_flight_altitudes(df)
    assert max(cleaned) < 20.0
    assert cleaned[2] == 10.5
    assert cleaned[3] == 10.5
    assert cleaned[4] == 11.0


@pytest.mark.skipif(not HAS_PANDAS_NUMPY, reason="pandas or numpy not installed")
def test_list_flight_logs_chronological_order(tmp_path):
    """Verifies that flight logs are sorted chronologically with newest flights first."""
    # Create mock flight logs with different timestamps
    f1 = tmp_path / "manta_flight_0001_20260901_100000.csv"
    f2 = tmp_path / "manta_flight_0002_20260907_120000.csv"
    f3 = tmp_path / "manta_flight_0003_20260921_090000.csv"
    f4 = tmp_path / "manta_flight_0004_20260921_110000.csv"

    for f in [f1, f2, f3, f4]:
        f.write_text("Record Number,Elapsed Time (s),Latitude,Longitude\n1,0.0,32.7,-16.8\n")

    logs = list_available_flight_logs(log_dir=str(tmp_path))
    assert len(logs) == 4
    # Expected order: newest first (f4, f3, f2, f1)
    basenames = [os.path.basename(l) for l in logs]
    assert basenames == [
        "manta_flight_0004_20260921_110000.csv",
        "manta_flight_0003_20260921_090000.csv",
        "manta_flight_0002_20260907_120000.csv",
        "manta_flight_0001_20260901_100000.csv",
    ]


@pytest.mark.skipif(not HAS_PYMAVLINK, reason="pymavlink not installed")
def test_handle_mission_planner_requests():
    """Verifies that handle_mission_planner_requests answers PARAM_REQUEST_LIST, MISSION_REQUEST_LIST, and GET_HOME."""
    from replay_flight_mission_planner import handle_mission_planner_requests

    # Set up loopback pair
    gcs = mavutil.mavlink_connection("udpin:127.0.0.1:14560")
    replayer = mavutil.mavlink_connection("udpout:127.0.0.1:14560", source_system=1, source_component=1)

    # Replayer sends heartbeat so GCS registers remote client IP/port
    replayer.mav.heartbeat_send(1, 0, 0, 0, 0)
    gcs.recv_msg()

    # 1. PARAM_REQUEST_LIST
    gcs.mav.param_request_list_send(1, 1)
    time.sleep(0.02)
    handle_mission_planner_requests(replayer, home_lat_e7=327000000, home_lon_e7=-168000000, home_alt_mm=10000)

    received_params = []
    start_t = time.time()
    while time.time() - start_t < 1.0:
        msg = gcs.recv_msg()
        if msg:
            if msg.get_type() == "PARAM_VALUE":
                received_params.append(msg)
                if msg.param_index + 1 == msg.param_count:
                    break
        else:
            time.sleep(0.01)

    assert len(received_params) >= 5
    assert any("SYSID_SW_MREV" in str(p.param_id) for p in received_params)
    assert any("STAT_RUNTIME" in str(p.param_id) for p in received_params)

    # 2. MISSION_REQUEST_LIST (Home waypoint protocol)
    gcs.mav.mission_request_list_send(1, 1)
    time.sleep(0.02)
    handle_mission_planner_requests(replayer, home_lat_e7=327000000, home_lon_e7=-168000000, home_alt_mm=10000)

    start_t = time.time()
    mission_cnt_received = False
    while time.time() - start_t < 1.0:
        msg = gcs.recv_msg()
        if msg and msg.get_type() == "MISSION_COUNT":
            assert msg.count == 1  # 1 waypoint representing Home
            mission_cnt_received = True
            break
        time.sleep(0.01)
    assert mission_cnt_received is True

    # 3. MISSION_REQUEST_INT (Request Waypoint 0 / Home)
    gcs.mav.mission_request_int_send(1, 1, 0)
    time.sleep(0.02)
    handle_mission_planner_requests(replayer, home_lat_e7=327000000, home_lon_e7=-168000000, home_alt_mm=10000)

    start_t = time.time()
    item_received = False
    while time.time() - start_t < 1.0:
        msg = gcs.recv_msg()
        if msg and msg.get_type() == "MISSION_ITEM_INT":
            assert msg.seq == 0
            assert msg.x == 327000000
            assert msg.y == -168000000
            item_received = True
            break
        time.sleep(0.01)
    assert item_received is True

    # 4. COMMAND_LONG (MAV_CMD_DO_SET_HOME - cmd 179)
    gcs.mav.command_long_send(1, 1, 179, 0, 1.0, 0, 0, 0, 0, 0, 0)
    time.sleep(0.02)
    h_lat, h_lon, h_alt = handle_mission_planner_requests(
        replayer, home_lat_e7=327000000, home_lon_e7=-168000000, home_alt_mm=10000,
        cur_lat_e7=327100000, cur_lon_e7=-168100000, cur_alt_mm=15000
    )
    assert h_lat == 327100000
    assert h_lon == -168100000

    start_t = time.time()
    cmd_ack_received = False
    home_pos_received = False
    while time.time() - start_t < 1.0:
        msg = gcs.recv_msg()
        if msg:
            if msg.get_type() == "COMMAND_ACK" and msg.command == 179:
                assert msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
                cmd_ack_received = True
            elif msg.get_type() == "HOME_POSITION":
                assert msg.latitude == 327100000
                home_pos_received = True
            if cmd_ack_received and home_pos_received:
                break
        time.sleep(0.01)
    assert cmd_ack_received is True
    assert home_pos_received is True


@pytest.mark.skipif(not HAS_PYMAVLINK, reason="pymavlink not installed")
def test_send_mavlink_frame_gps_and_sensors_health():
    """Verifies that send_mavlink_frame provides 1.0m HDOP (eph=100) and includes GPS sensor in SYS_STATUS."""
    from replay_flight_mission_planner import send_mavlink_frame

    gcs = mavutil.mavlink_connection("udpin:127.0.0.1:14561")
    replayer = mavutil.mavlink_connection("udpout:127.0.0.1:14561", source_system=1, source_component=1)

    now = time.time()
    send_mavlink_frame(
        replayer, time_boot_ms=1000, now=now,
        lat=32.7259, lon=-16.8850, alt_m=50.0,
        roll=0.0, pitch=2.0, yaw=90.0,
        airspeed=15.0, gnd_spd=15.0, vx_mps=0.0, vy_mps=15.0,
        climb_rate=0.5, throttle_pct=50, battery_v=15.2,
        satellites=16, fix_type=3, rc_lost=False
    )

    received = {}
    start_t = time.time()
    while time.time() - start_t < 1.0:
        msg = gcs.recv_msg()
        if msg:
            received[msg.get_type()] = msg
            if "GPS_RAW_INT" in received and "SYS_STATUS" in received:
                break
        else:
            time.sleep(0.01)

    assert "GPS_RAW_INT" in received
    gps_raw = received["GPS_RAW_INT"]
    assert gps_raw.fix_type == 3
    assert gps_raw.eph == 100  # 1.00 m HDOP (optimal, eliminates 'Bad GPS Pos')
    assert gps_raw.epv == 150  # 1.50 m VDOP
    assert gps_raw.satellites_visible >= 12

    assert "SYS_STATUS" in received
    sys_status = received["SYS_STATUS"]
    assert (sys_status.onboard_control_sensors_present & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS) != 0
    assert (sys_status.onboard_control_sensors_enabled & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS) != 0
    assert (sys_status.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS) != 0
