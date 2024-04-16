from pymavlink import mavutil
# Import ardupilotmega module for MAVLink 1
from pymavlink.dialects.v10 import ardupilotmega as mavlink1

# Import common module for MAVLink 2
from pymavlink.dialects.v20 import common as mavlink2

connection_string = 'tcp:127.0.0.1:5760'

vehicle = mavutil.mavlink_connection(connection_string)

while True:
    # Send a request for GLOBAL_POSITION_INT data
    vehicle.mav.request_data_stream_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION,
        1,  # Hz
        1   # Start sending
    )

    msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if msg:
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt = msg.alt / 1e3  # Altitude in meters
        print(f"GPS Location - Latitude: {lat}, Longitude: {lon}, Altitude: {alt} meters")
        break
while True:
    msg = vehicle.recv_match(type='HEARTBEAT', blocking=True)
    if msg:
        print(msg.autopilot==mavutil.mavlink.MAV_AUTOPILOT_PIXHAWK)
        break

vehicle.close()