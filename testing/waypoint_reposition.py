import time
from pymavlink import mavutil
from math import nan

def goto_waypoint(master, target_lat, target_lon, target_alt=10):
    master.mav.command_int_send(master.target_system,
                                master.target_component,
                                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                mavutil.mavlink.MAV_CMD_DO_REPOSITION,
                                0, 0,
                                -1.0, 1.0, nan, nan,
                                target_lat, target_lon, target_alt)

    msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)
    print('Heading to waypoint!')
    while True:
        msg = master.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)
        print(msg)
        time.sleep(1)

def ack(conn, keyword):
    print("---Message Received: " + str(conn.recv_match(type=keyword, blocking=True)))


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

# Takeoff
target_altitude = 10
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0, 0, 0, 0, 0, 0, target_altitude)
msg = master.recv_match(type="COMMAND_ACK", blocking=True)
print(msg)

#Messages for takeoff
while True:
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    print("Altitude: ", msg.relative_alt)
    if msg.relative_alt > target_altitude*1000*0.98:
        break
    time.sleep(1)
print(f"Reached target altitude of {target_altitude} metres!")


print(f'Waypoint: {int(-35.3638140* 10**7)}, {int(149.1658000* 10**7)}')

goto_waypoint(master, int(-35.3638140 * 1e7), int(149.1658000 * 1e7))

# Landing
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0,
    0, 0, 0, 0, 0, 0, 0)
msg = master.recv_match(type="COMMAND_ACK", blocking=True)
print(msg)

#Messages for landing
while True:
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    print("Altitude: ", msg.relative_alt)
    if msg.relative_alt<10:
        break
    time.sleep(1)
print('Vehicle Landed!')

# Disarm
print('Disarming...')
master.arducopter_disarm()
msg = master.recv_match(type="COMMAND_ACK", blocking=True)
print(msg)
print("Disarmed!")

#End mission!!!

'''
    master.mav.command_int_send(master.target_system,
                                master.target_component,
                                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                mavutil.mavlink.MAV_CMD_DO_REPOSITION,
                                0, 0,
                                -1.0, 1.0, 0.0, nan,
                                target_lat, target_lon, target_alt)
'''

'''
    master.mav.command_long_send(master.target_system,
                                master.target_component,
                                mavutil.mavlink.MAV_CMD_DO_REPOSITION,
                                0,
                                -1.0, 1.0, 0.0, nan,
                                target_lat, target_lon, target_alt)
'''