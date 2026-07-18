from pymavlink import mavutil
import time
from Simulator.examples.aux_code.general.basic_aux_code import get_all_telemetry

# Setup connection
connection = mavutil.mavlink_connection('tcp:127.0.0.1:5762')

print("Waiting for SITL...")
connection.wait_heartbeat()
print("Connected! Starting telemetry stream...\n")

try:
    while True:
        data = get_all_telemetry(connection)

        if data:
            print("========================================")
            print(f"ORIENTATION | Roll: {data['roll']:>6.2f}º | Pitch: {data['pitch']:>6.2f}º | Yaw: {data['yaw']:>6.2f}º")
            print(f"ALTITUDE    | Relative: {data['alt']:>6.2f} m")
            print(f"SPEED       | Ground: {data['groundspeed']:>6.2f} m/s | Air: {data['airspeed']:>6.2f} m/s")
            print(f"STATUS      | Heading: {data['heading']:>3}º | Throttle: {data['throttle']:>3}%")

        time.sleep(0.2) 

except KeyboardInterrupt:
    print("\nTelemetry stream stopped.")