from pymavlink import mavutil
from aux_code.general.basic_aux_code import arm_plane, get_all_telemetry, set_controls, normalize_direction
from aux_code.control.pid import *
import time

KP = 0.6
KI = 0.35
KD = 0.1

elevator_pid = PID(kp=4.5, ki=0.8, kd=0.1, setpoint=0, output_limits=(-500, 500), ki_limit=100)
roll_pid = PID(kp=0.2, ki=0.7, kd=1, setpoint=0, output_limits=(-500, 500), ki_limit=100)
direction_roll_pid = PID(kp=KP, ki=KI, kd=KD, setpoint=0, output_limits=(-25, 25), ki_limit=7)

# 1. Connecting to the simulator via MAVLink (TCP port 5762 for SERIAL1)
connection = mavutil.mavlink_connection('tcp:127.0.0.1:5762')

# Waiting for the SITL heartbeat
print("Waiting for SITL communication...")
connection.wait_heartbeat()
print("Connected to Simulator!")
time.sleep(2)

# Main Control Loop

arm_plane(connection)
print("Ready to fly!")
time.sleep(5)

while True:
    # 1. Receive sensor data (unpacking all 3 return values)
    all_data = get_all_telemetry(connection)
    roll, pitch, altitude = all_data["roll"], all_data["pitch"], all_data["alt"]
    yaw = all_data["yaw"]

    print(f"Current Pitch: {pitch:.2f}° [{elevator_pid.setpoint:.2f}] | Current Roll: {roll:.2f}° [{roll_pid.setpoint:.2f}] | Yaw: {yaw:.2f}° | Altitude: {altitude:.1f}m")

    # 2. Decision Logic (Commented out placeholders)
    target_direction = 0
    if altitude < 5:
        target_pitch = 0
    elif altitude > 100:
        target_pitch = 5
        target_direction = 90
    else:
        target_pitch = 10

    elevator_pid.soft_set_target(target_pitch, iteration=0.2)
    target_roll = direction_roll_pid.update(-normalize_direction(yaw, target_direction))
    roll_pid.soft_set_target(target_roll, iteration=2)

    pitch_pwm = round(1500 + elevator_pid.update(pitch))
    roll_pwm = round(1500 + roll_pid.update(roll))

    print(f"pitch_pwm: {pitch_pwm} | roll_pwm: {roll_pwm}")
    # 3. Send commands
    set_controls(connection, roll_pwm, pitch_pwm, 1700)

    time.sleep(0.05)  # Execute at 20Hz



