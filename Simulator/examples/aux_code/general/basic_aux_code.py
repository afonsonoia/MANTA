import math
import time
from pymavlink import mavutil

def arm_plane(connection):
    """Sends the ARM command to the flight controller with a 10-second countdown"""

    # MAV_CMD_COMPONENT_ARM_DISARM is MAVLink command ID 400
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # Confirmation (0 means first transmission)
        1,  # Param 1: 1 to Arm, 0 to Disarm
        0, 0, 0, 0, 0, 0  # Params 2 to 7 are not used for this command
    )

    # Wait for the vehicle's response to ensure it accepted the command
    msg = connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)

    if msg:
        # result == 0 means MAV_RESULT_ACCEPTED
        if msg.result == 0:
            print("SUCCESS: The plane is ARMED! Watch the propeller!")
        else:
            print(f"FAILED: The plane rejected the Arm command. Error code: {msg.result}")
    else:
        print("FAILED: No response received from the plane.")


def set_controls(connection, roll_pwm, pitch_pwm, throttle_pwm):
    """
    Sends RC Override values to the servos and motor.
    Standard Channels: 1: Roll, 2: Pitch, 3: Throttle, 4: Yaw
    PWM Values: 1000 (Min) to 2000 (Max). 1500 is Neutral.
    """
    connection.mav.rc_channels_override_send(
        connection.target_system,
        connection.target_component,
        roll_pwm,  # Channel 1
        pitch_pwm,  # Channel 2
        throttle_pwm,  # Channel 3
        65535,  # Channel 4 (Yaw) - 65535 means "ignore this channel"
        65535, 65535, 65535, 65535  # Channels 5-8 ignored
    )

def get_all_telemetry(connection):
    """
    Fetches the main flight variables from different MAVLink messages.
    """
    # 1. Orientation (Roll, Pitch, Yaw)
    msg_att = connection.recv_match(type='ATTITUDE', blocking=True)

    # 2. Altitude (Relative to home)
    msg_pos = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)

    # 3. Speeds and Heading (Standard pilot view)
    msg_hud = connection.recv_match(type='VFR_HUD', blocking=True)

    # Data Processing
    telemetry = {
        "roll": round(math.degrees(msg_att.roll), 1),
        "pitch": round(math.degrees(msg_att.pitch), 1),
        "yaw": round(math.degrees(msg_att.yaw), 1),
        "alt": msg_pos.relative_alt / 1000.0,  # Convert mm to meters
        "groundspeed": msg_hud.groundspeed,  # m/s
        "airspeed": msg_hud.airspeed,  # m/s
        "heading": msg_hud.heading,  # degrees (0-360)
        "throttle": msg_hud.throttle  # percentage (0-100)
    }

    return telemetry


def normalize_direction(current_direction:int, target_direction:int):
    direction = target_direction - current_direction
    if direction > 180:
        direction -= 360
    elif direction < -180:
        direction += 360
    return direction