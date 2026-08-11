import time
import math
from pymavlink import mavutil

import sys

def main():
    mode = "tcp"
    if len(sys.argv) > 1 and sys.argv[1].lower() == "udp":
        mode = "udp"

    print("=" * 60)
    print("  MAVLink Streamer for Mission Planner Virtual Horizon (TCP Server)")
    print("=" * 60)
    
    if mode == "tcp":
        print("Servidor TCP ativo na porta 14550...")
        print("No Mission Planner: Seleciona 'TCP' -> Host: 127.0.0.1 -> Port: 14550 -> Clica em Connect.")
        conn_str = 'tcpin:0.0.0.0:14550'
    else:
        print("A transmitir UDP para 127.0.0.1:14550...")
        print("No Mission Planner: Seleciona 'UDP' -> Port: 14550 -> Clica em Connect.")
        conn_str = 'udpout:127.0.0.1:14550'


    print("Press Ctrl+C to stop streaming.\n")

    mav = mavutil.mavlink_connection(conn_str, source_system=1, source_component=1)
    
    start_time = time.time()
    last_heartbeat = 0.0


    try:
        while True:
            now = time.time()
            elapsed = now - start_time
            time_boot_ms = int(elapsed * 1000) & 0xFFFFFFFF

            # Send Heartbeat, Sys Status, and Dummy Param at 2 Hz to prevent Mission Planner aggregation errors
            if (now - last_heartbeat) >= 0.5:
                last_heartbeat = now
                try:
                    mav.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_FIXED_WING,
                        mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                        mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED | mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED,
                        0,
                        mavutil.mavlink.MAV_STATE_ACTIVE
                    )
                    mav.mav.sys_status_send(
                        0, 0, 0, 500, 12500, -1, -1, 0, 0, 0, 0, 0, 0
                    )
                    # Dummy parameter ensures Mission Planner's param dictionary is non-empty
                    mav.mav.param_value_send(
                        b"STAT_RUNTIME",
                        1.0,
                        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
                        1,
                        0
                    )
                except Exception:
                    pass

            # Read any incoming requests from Mission Planner
            try:
                msg = mav.recv_msg()
                if msg is not None:
                    m_type = msg.get_type()
                    if m_type in ['PARAM_REQUEST_LIST', 'PARAM_REQUEST_READ']:
                        mav.mav.param_value_send(
                            b"STAT_RUNTIME",
                            1.0,
                            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
                            1,
                            0
                        )
                    elif m_type == 'COMMAND_LONG':
                        mav.mav.command_ack_send(msg.command, mavutil.mavlink.MAV_RESULT_ACCEPTED)
            except Exception:
                pass

            # Generate smooth test motion: Pitch +/- 20 deg, Roll +/- 30 deg
            roll_deg = 30.0 * math.sin(elapsed * 1.5)
            pitch_deg = 20.0 * math.cos(elapsed * 1.0)
            yaw_deg = (elapsed * 10.0) % 360.0

            mav.mav.attitude_send(
                time_boot_ms,
                math.radians(roll_deg),
                math.radians(pitch_deg),
                math.radians(yaw_deg),
                0.0, 0.0, 0.0 # angular speeds
            )


            print(f"\r[Streaming to Mission Planner] Roll: {roll_deg:+6.1f}° | Pitch: {pitch_deg:+6.1f}° | Yaw: {yaw_deg:5.1f}°", end="")
            time.sleep(0.02) # 50 Hz refresh rate
            
    except KeyboardInterrupt:
        print("\n[Stopped MAVLink stream.]")

if __name__ == '__main__':
    main()
