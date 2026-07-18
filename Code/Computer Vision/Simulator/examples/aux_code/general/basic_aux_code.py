import math
import time
from pymavlink import mavutil

def arm_plane(connection):
    """Sends the ARM command"""
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )

    msg = connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)

    if msg:
        if msg.result == 0:
            print("SUCCESS: Plane ARMED!")
        else:
            print(f"FAILED: Plane rejected Arm command. Error: {msg.result}")
    else:
        print("FAILED: No response.")

def set_controls(connection, roll_pwm, pitch_pwm, throttle_pwm):
    """Sends RC Override values"""
    connection.mav.rc_channels_override_send(
        connection.target_system,
        connection.target_component,
        roll_pwm, pitch_pwm, throttle_pwm,
        65535, 65535, 65535, 65535, 65535 
    )

def get_all_telemetry(connection):
    """Fetches latest flight variables"""
    while connection.recv_match(blocking=False):
        pass

    msg_att = connection.messages.get('ATTITUDE')
    msg_pos = connection.messages.get('GLOBAL_POSITION_INT')
    msg_hud = connection.messages.get('VFR_HUD')

    if not msg_att or not msg_pos or not msg_hud:
        return None

    telemetry = {
        "roll": round(math.degrees(msg_att.roll), 1),
        "pitch": round(math.degrees(msg_att.pitch), 1),
        "yaw": round(math.degrees(msg_att.yaw), 1),
        "alt": msg_pos.relative_alt / 1000.0,
        "groundspeed": msg_hud.groundspeed,
        "airspeed": msg_hud.airspeed,
        "heading": msg_hud.heading,
        "throttle": msg_hud.throttle
    }

    return telemetry

def normalize_direction(current_direction:int, target_direction:int):
    direction = target_direction - current_direction
    if direction > 180:
        direction -= 360
    elif direction < -180:
        direction += 360
    return direction