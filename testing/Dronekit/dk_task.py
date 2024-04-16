import dronekit_sitl
# sitl = dronekit_sitl.start_default()

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import socket
import math
import argparse


def arm_and_takeoff(vehicle, altitude=5):
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialize...")
        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)
    vehicle.simple_takeoff(altitude)
    while True:
        print("Vehicle rising in altitude...")
        print("Current altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

    vehicle.flush()


def land(vehicle):
    vehicle.mode = VehicleMode("LAND")
    while not vehicle.armed:
        print("Vehicle landing...")
        time.sleep(1)
    print("Landing complete, altitude: ", vehicle.location.global_relative_frame.alt)

def euc_diff(a: LocationGlobalRelative, b: LocationGlobalRelative):
    return math.sqrt((a.lat - b.lat)**2 + (a.lon - b.lon)**2)

def goto_waypoint(vehicle, loc):
    vehicle.simple_goto(loc)
    while euc_diff(loc, vehicle.location.global_relative_frame) > 0.5:
        print("Vehicle heading to waypoint: ", loc)
        print("Distance to waypoint: ", euc_diff(loc, vehicle.location.global_relative_frame))
        time.sleep(2)
    print("Vehicle reached waypoint: ", loc)


parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect')     # tcp:127.0.0.1:5760
args = parser.parse_args()

connection_string = args.connect

vehicle = connect(connection_string, wait_ready=True)

print("Initial:\n-----------------------------------------------------")
print("Vehicle mode: ", vehicle.mode.name)
print("Coordinates: ", vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
print(vehicle.location.global_relative_frame)
print("Vehicle Altitude: ", vehicle.location.global_relative_frame.alt)
print("Vehicle Armed: ", vehicle.armed)

arm_and_takeoff(vehicle, 10)

way1 = LocationGlobalRelative(-35.364114, 149.166022, 10)
goto_waypoint(vehicle, way1)

way2 = LocationGlobalRelative(-37.364114, 154.166022, 10)
goto_waypoint(vehicle, way2)

way3 = LocationGlobalRelative(-45.364114, 124.166022, 10)
goto_waypoint(vehicle, way3)

way4 = LocationGlobalRelative(-34.364114, 149.166022, 2)
goto_waypoint(vehicle, way4)

land(vehicle)

print("Final:\n-----------------------------------------------------")
print("Vehicle mode: ", vehicle.mode.name)
print("Coordinates: ", vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
print(vehicle.location.global_relative_frame)
print("Vehicle Altitude: ", vehicle.location.global_relative_frame.alt)
print("Vehicle Armed: ", vehicle.armed)
vehicle.close()