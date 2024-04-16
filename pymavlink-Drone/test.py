from drone_class import Drone

connection_string = 'udp:127.0.0.1:14550'

master = Drone(connection_string=connection_string)

try:
    master.get_attitude()
    master.get_speed()

    master.arm_and_takeoff(5, skip_prearm_checks=True)

    master.goto_waypoint(-35.3645384, 149.1648531, 10)

    master.change_speed(10)

    master.goto_waypoint(-35.3633572, 149.1652072, 5)

    master.change_altitude(10)

    master.land()

except KeyboardInterrupt as e:
    print(f'KeyBoard Interrupt Detected ---> Emergency Landing Initiated!')
    master.land()