import time
import math
from math import sin, cos, acos, nan
from pymavlink import mavutil

def euc_diff(current_lat, current_lon, target_lat, target_lon):
    current_lat = current_lat / 180 * math.pi
    current_lon = current_lon / 180 * math.pi
    target_lat = target_lat / 180 * math.pi
    target_lon = target_lon / 180 * math.pi

    distance = sin(current_lat) * sin(target_lat) + cos(current_lat) * cos(target_lat) * cos(target_lon - current_lon)
    distance = acos(distance) * 6371 * 1000

    return distance

def euc_difff(current_lat, current_lon, target_lat, target_lon):
    return math.sqrt((current_lat - target_lat) ** 2 + (current_lon - target_lon) ** 2)

    return distance

def goto_waypoint(master, target_lat, target_lon, target_alt=10):
    master.mav.command_int_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0, 0,
        0, 0, 0, 0, int(target_lat*1e7), int(target_lon*1e7), target_alt, True)

    msg = master.recv_match(type="COMMAND_ACK")
    print(msg)
    print('Heading to waypoint!')
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        current_lat = msg.lat / 1e7
        current_lon = msg.lon / 1e7
        print(f'Target: {target_lat}, {target_lon}')
        print(f'Current: {current_lat}, {current_lon}')
        distance = euc_diff(current_lat, current_lon, target_lat, target_lon)
        print(f'Distance to waypoint: {distance * 10000} metres')
        if distance < 0.0001:
            print(*(['*']*20))
            print('Reached waypoint!!!')
            break
        time.sleep(2)

# Create the connection
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
# Wait a heartbeat before sending commands
master.wait_heartbeat()
print(f'Heartbeat from system {master.target_system}, component {master.target_component}')

# https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM
# Arm
print("Arming...")
master.arducopter_arm()

# wait until arming confirmed (can manually check with master.motors_armed())
#master.motors_armed_wait()
msg = master.recv_match(type="COMMAND_ACK", blocking=True)
print(msg)
print('Armed!')

#time.sleep(5)

target_altitude = 10

master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0, 0, 0, 0, 0, 0, target_altitude)
msg = master.recv_match(type="COMMAND_ACK", blocking=True)
print(msg)

while True:
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    print("Altitude: ", msg.relative_alt)
    if msg.relative_alt > target_altitude*1000*0.98:
        break
    time.sleep(1)

print(f"Reached target altitude of {target_altitude} metres!")

#Go to waypoint
target_lat = -35.3638140
target_lon = 149.1658000

goto_waypoint(master, target_lat, target_lon)

master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0,
    0, 0, 0, 0, 0, 0, 0)
msg = master.recv_match(type="COMMAND_ACK", blocking=True)
print(msg)

while True:
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    print("Altitude: ", msg.relative_alt)
    if msg.relative_alt<10:
        break
    time.sleep(1)

print('Vehicle Landed!')
print('Disarming...')
# Disarm
master.arducopter_disarm()

# wait until disarming confirmed
#master.motors_disarmed_wait()
msg = master.recv_match(type="COMMAND_ACK", blocking=True)
print(msg)
print("Disarmed!")