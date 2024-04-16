import time
from pymavlink import mavutil

# Mavproxy command line: < mavproxy.exe --master=tcp:127.0.0.1:5760 --out=127.0.0.1:14550 --out=127.0.0.1:14551 --out=127.0.0.1:14552 >

def goto_waypoint(master, target_lat, target_lon, target_alt):
    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(0, master.target_system, master.target_component,
                                                                                  mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, int(0b110111111000),
                                                                                  target_lat, target_lon, target_alt, 0, 0, 0, 0, 0, 0, 0, 0))

    print('Heading to waypoint!')
    start = False
    while True:
        msg = master.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)
        print(f'Distance to waypoint: {msg.wp_dist}')
        print(msg)
        if msg.wp_dist > 0:
            start = True
        elif msg.wp_dist == 0 and start:
            break
        time.sleep(3)
    print("\nWaypoint reached!!!\n")

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

target_altitude = 5

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
    print(msg)
    if msg.relative_alt > target_altitude*1000*0.98:
        break
    time.sleep(1)

print(f"Reached target altitude of {target_altitude} metres!")

#Go to waypoint
target_lat = -35.3645384
target_lon = 149.1648531

target_lat = int(target_lat * 1e7)
target_lon = int(target_lon * 1e7)

goto_waypoint(master, target_lat, target_lon, target_altitude)

goto_waypoint(master, int(-35.3633572 * 1e7), int(149.1652072 * 1e7), target_altitude)

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