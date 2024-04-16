import dronekit_sitl
# sitl = dronekit_sitl.start_default()

from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
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

parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect')
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
land(vehicle)

print("Final:\n-----------------------------------------------------")
print("Vehicle mode: ", vehicle.mode.name)
print("Coordinates: ", vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
print(vehicle.location.global_relative_frame)
print("Vehicle Altitude: ", vehicle.location.global_relative_frame.alt)
print("Vehicle Armed: ", vehicle.armed)
vehicle.close()