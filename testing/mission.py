import time
from pymavlink import mavutil
from math import nan

class mission_item:
    def __init__(self, seq, current, x, y, z):
        self.seq = seq
        self.frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        self.command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        self.current = current
        self.auto = 1
        self.param1 = 0.0
        self.param2 = 2.0
        self.param3 = 20.0
        self.param4 = nan
        self.param5 = x
        self.param6 = y
        self.param7 = z
        self.mission_type = 0

def upload_mission(conn, mission_items):
    n = len(mission_items)
    print("---Sending Message Out")

    conn.mav.mission_count_send(conn.target_system, conn.target_component, n)

    ack(conn, "MISSION_REQUEST")

    for waypoint in mission_items:
        print("---Creating a Waypoint")

        conn.mav.mission_item_send(conn.target_system, conn.target_component, waypoint.seq, waypoint.frame, waypoint.command, waypoint.current, waypoint.auto,
                                   waypoint.param1, waypoint.param2, waypoint.param3, waypoint.param4, waypoint.param5, waypoint.param6, waypoint.param7,
                                   waypoint.mission_type)

        if waypoint != mission_items[n-1]:
            ack(conn, "MISSION_REQUEST")

    ack(conn, "MISSION_ACK")

def start_mission(conn):
    print("---Mission Start")

    conn.mav.command_long_send(conn.target_system, conn.target_component, mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0, 0, 0, 0, 0, 0, 0)
    ack(conn, "COMMAND_ACK")

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
target_altitude = 5
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


#Start Mission
mission_waypoints = []
mission_waypoints.append(mission_item(0, 0, -35.3638209, 149.1676211, target_altitude*2))
mission_waypoints.append(mission_item(1, 0, -35.3638209, 149.1676211, target_altitude*2))
mission_waypoints.append(mission_item(2, 0, -35.3645384, 149.1648531, target_altitude*3))
mission_waypoints.append(mission_item(3, 0, -35.3633572, 149.1652072, target_altitude*4))


upload_mission(conn=master, mission_items=mission_waypoints)

start_mission(conn=master)

for mission_item in mission_waypoints[1:]:
    print("---Message Read: " + str(master.recv_match(type="MISSION_ITEM_REACHED", blocking=True)))

conn.mav.mission_clear_all_send(conn.target_system, conn.target_component)
ack(conn, "MISSION_ACK")

upload_mission(conn=master, mission_items=mission_waypoints)

start_mission(conn=master)

for mission_item in mission_waypoints[1:]:
    print("---Message Read: " + str(master.recv_match(type="MISSION_ITEM_REACHED", blocking=True)))

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
time.sleep(2)
master.arducopter_disarm()
msg = master.recv_match(type="COMMAND_ACK", blocking=True)
print(msg)
print("Disarmed!")

#End mission!!!