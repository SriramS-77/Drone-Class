"""
Trying to set mode to 'GUIDED' with mavlink command
"""
import time
# Import ardupilotmega module for MAVLink 1
from pymavlink.dialects.v10 import ardupilotmega as mavlink1

# Import common module for MAVLink 2
from pymavlink.dialects.v20 import common as mavlink2

# Import mavutil
from pymavlink import mavutil
#import pymavlink.dialects.v20.all as dialect

# Create the connection
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
# Wait a heartbeat before sending commands
master.wait_heartbeat()
print(f'Heartbeat from system {master.target_system}, component {master.target_component}')

FLIGHT_MODE = 'GUIDED'
print("Modes: ", master.mode_mapping().keys())
'''
# create change mode message
set_mode_message = dialect.MAVLink_command_long_message(
    target_system=master.target_system,
    target_component=master.target_component,
    command=dialect.MAV_CMD_DO_SET_MODE,
    confirmation=0,
    param1=dialect.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    param2=(master.mode_mapping())[FLIGHT_MODE],
    param3=0,
    param4=0,
    param5=0,
    param6=0,
    param7=0
)

master.mav.send(set_mode_message)
msg = master.recv_match(type="COMMAND_ACK")
print(msg)
'''

# https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM
# Arm
# master.arducopter_arm() or:
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)

# wait until arming confirmed (can manually check with master.motors_armed())
print("Waiting for the vehicle to arm")
print(master.motors_armed())
#master.motors_armed_wait()
msg = master.recv_match(type="COMMAND_ACK", blocking=True)
print(msg)
print('Armed!')

#time.sleep(5)

master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0, 0, 0, 0, 0, 0, 10)
msg = master.recv_match(type="COMMAND_ACK", blocking=True)
print(msg)

time.sleep(10)

master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0,
    0, 0, 0, 0, 0, 0, 0)
msg = master.recv_match(type="COMMAND_ACK", blocking=True)
print(msg)
#time.sleep(25)
while True:
    msg = master.recv_match(blocking=True)
    print(msg)
    time.sleep(1)
# Disarm
# master.arducopter_disarm() or:

master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0)

# wait until disarming confirmed
#master.motors_disarmed_wait()
msg = master.recv_match(type="COMMAND_ACK", blocking=True)
print(msg)
print("Disarmed!")