from pymavlink import mavutil
import time
import math
from Simulator.examples.aux_code.general.basic_aux_code import arm_plane, set_controls

last_roll_value = 1500

# 1. Connecting to the simulator via MAVLink (TCP port 5762 for SERIAL1)
connection = mavutil.mavlink_connection('tcp:127.0.0.1:5762')

# Waiting for the SITL heartbeat
print("Waiting for SITL communication...")
connection.wait_heartbeat()
print("Connected to Simulator!")
time.sleep(2)


def get_telemetry():
    """Retrieves attitude (Roll/Pitch) and relative altitude data"""

    # 1. Catch the Attitude message
    msg_att = connection.recv_match(type='ATTITUDE', blocking=True)

    # 2. Catch the Global Position message (contains GPS and Barometer fused data)
    msg_pos = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)

    # 'relative_alt' is in millimeters. Divide by 1000 to get meters.
    altitude_meters = msg_pos.relative_alt / 1000.0

    return msg_att.roll, msg_att.pitch, altitude_meters


def example_roll_control(target_roll: float, real_roll: float):
    global last_roll_value
    """Simple threshold-based control logic example"""

    if real_roll < target_roll - 10:
        roll_command = last_roll_value + 1  # Turn right
    elif real_roll > target_roll + 10:
        roll_command = last_roll_value - 1  # Turn left
    else:
        if target_roll == 0:
            roll_command = 1500
        else:
            roll_command = last_roll_value

    last_roll_value = roll_command
    return roll_command


# Main Control Loop
try:
    arm_plane(connection)

    while True:
        # 1. Receive sensor data (unpacking all 3 return values)
        roll, pitch, altitude = get_telemetry()

        # Convert radians to degrees for easier handling
        current_roll_deg = roll * (180 / math.pi)

        print(f"Current Roll: {current_roll_deg:.2f}° | Altitude: {altitude:.1f}m")

        # 2. Decision Logic (Commented out placeholders)
        if altitude < 200:
            target_roll = 0.0
        else:
            target_roll = 50
        roll_pwm = example_roll_control(target_roll, current_roll_deg)

        # 3. Send commands
        set_controls(connection, roll_pwm, 1500, 1700)

        time.sleep(0.1)  # Execute at 10Hz

except KeyboardInterrupt:
    print("\nControl interrupted by user.")