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

def goto_waypoint(conn: mavutil.mavudp, target_latitude, target_longitude, target_altitude):
    print('*'*20)
    waypoint = mission_item(0, 0, target_latitude, target_longitude, target_altitude)

    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    print(msg)
    intermediate_lat = int((waypoint.param5 + msg.lat) / 2)
    intermediate_lon = int((waypoint.param6 + msg.lon) / 2)

    conn.mav.mission_count_send(conn.target_system, conn.target_component, 2)
    ack(conn, "MISSION_REQUEST")
    print("---Creating waypoint with sequence:", waypoint.seq)
    conn.mav.mission_item_send(conn.target_system, conn.target_component, waypoint.seq, waypoint.frame, waypoint.command,
                               waypoint.current, waypoint.auto,
                               waypoint.param1, waypoint.param2, waypoint.param3, waypoint.param4,
                               waypoint.param5, waypoint.param6, waypoint.param7,
                               waypoint.mission_type)
    ack(conn, "MISSION_REQUEST")
    print("---Creating waypoint with sequence:", waypoint.seq+1)
    conn.mav.mission_item_send(conn.target_system, conn.target_component, waypoint.seq + 1, waypoint.frame, waypoint.command,
                               waypoint.current, waypoint.auto,
                               waypoint.param1, waypoint.param2, waypoint.param3, waypoint.param4,
                               waypoint.param5, waypoint.param6, waypoint.param7,
                               waypoint.mission_type)

    ack(conn, "MISSION_ACK")
    print("---Mission Start")
    conn.mav.command_long_send(conn.target_system, conn.target_component, mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0, 1, 0, 0, 0, 0, 0)
    ack(conn, "COMMAND_ACK")
    print("---Message Read: " + str(conn.recv_match(type="MISSION_ITEM_REACHED", blocking=True)))
    conn.mav.mission_clear_all_send(conn.target_system, conn.target_component)
    ack(conn, "MISSION_ACK")
    return


def ack(conn, keyword):
    print("---Message Received: " + str(conn.recv_match(type=keyword, blocking=True)))

def arm_and_takeoff(conn, target_altitude):
    # Pre-arm checks
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS,
        0, 0, 0, 0, 0, 0, 0, 0)
    msg = master.recv_match(type="COMMAND_ACK", blocking=True)

    print("Arming...")
    conn.arducopter_arm()
    msg = master.recv_match(type="COMMAND_ACK", blocking=True)
    print(msg)
    print('Armed!')

    # Takeoff
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0, target_altitude)
    msg = master.recv_match(type="COMMAND_ACK", blocking=True)
    print(msg)

    # Messages for takeoff
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        print("Altitude: ", msg.relative_alt)
        if msg.relative_alt > target_altitude * 1000 * 0.98:
            break
        time.sleep(1)
    print(f"Reached target altitude of {target_altitude} metres!")

    return

def land_and_disarm(conn):
    # Landing
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0, 0, 0, 0)
    msg = master.recv_match(type="COMMAND_ACK", blocking=True)
    print(msg)

    # Messages for landing
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        print("Altitude: ", msg.relative_alt)
        if msg.relative_alt < 10:
            break
        time.sleep(1)
    print('Vehicle Landed!')

    # Disarm
    print('Disarming...')
    master.arducopter_disarm()
    msg = master.recv_match(type="COMMAND_ACK", blocking=True)
    print(msg)
    print("Disarmed!")

    return


# Create the connection
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
# Wait a heartbeat before sending commands
master.wait_heartbeat()
print(f'Heartbeat from system {master.target_system}, component {master.target_component}')

alt = 5
arm_and_takeoff(master, alt)
goto_waypoint(conn=master, target_latitude=-35.3645384, target_longitude=149.1648531, target_altitude=alt)
goto_waypoint(conn=master, target_latitude=-35.3638209, target_longitude=149.1676211, target_altitude=alt*2)
goto_waypoint(conn=master, target_latitude=-35.3633572, target_longitude=149.1652072, target_altitude=alt*3)
land_and_disarm(master)