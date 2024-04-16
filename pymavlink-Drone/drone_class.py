from pymavlink import mavutil
import time

"""
drone is the primary class, representing the vehicle to command.
"""


class Drone:
    def __init__(self, connection_string):
        """
        :param connection_string: A string, representing the path or port through which the computer connects with and
                                  autonomously controls the vehicle.
        :return: nothing
        """
        self.__conn = mavutil.mavlink_connection(connection_string)
        self.__conn.wait_heartbeat()
        print(f'Connection established to drone!')
        print(f'Heartbeat received from system {self.__conn.target_system}, component {self.__conn.target_component}!')
        msg = self.__conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        self.lat, self.lon, self.alt = msg.lat, msg.lon, msg.alt // 1000
        print(f'\nHome location coordinates: {self.lat}, {self.lon}, {self.alt}\n')
    
    def pre_arm_checks(self):
        """
        Executes pre-arm safety checks.

        :return: nothing
        """
        print(f'Doing pre-arm checks...')
        self.__conn.mav.command_long_send(
            self.__conn.target_system,
            self.__conn.target_component,
            mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS,
            0, 0, 0, 0, 0, 0, 0, 0)
# Confirmation Message
        msg = self.__conn.recv_match(type="COMMAND_ACK", blocking=True)
        print(msg)

    def arm(self, skip_prearm_checks = False):
        """
        Arms the vehicle. Might perform pre-arm safety checks, depending on the command.

        :param skip_prearm_checks: This parameter controls whether to perform the pre-arm safety checks or to skip them.
        :return: nothing
        """
        if not skip_prearm_checks:
            self.pre_arm_checks()
        print(f'Arming...')

        self.__conn.mav.command_long_send(
            self.__conn.target_system,
            self.__conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0)
# Confirmation Message
        msg = self.__conn.recv_match(type="COMMAND_ACK", blocking=True)
        print(msg)
        print('Armed!')
    
    def disarm(self):
        """
        Disarms the vehicle.

        :return: nothing
        """
        print(f'Disarming...')
        self.__conn.mav.command_long_send(
            self.__conn.target_system,
            self.__conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0)
# Confirmation Message
        msg = self.__conn.recv_match(type="COMMAND_ACK", blocking=True)
        print(msg)
        print("Disarmed!")

    def force_disarm(self):
        """
        Forcefully disarms the vehicle, even when it is in the air.
        Can damage the vehicle.

        :return: nothing
        """
        print(f'Force-Disarming the drone...')
        self.__conn.mav.command_long_send(
            self.__conn.target_system,
            self.__conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 21196, 0, 0, 0, 0, 0)
        msg = self.__conn.recv_match(type="COMMAND_ACK", blocking=True)
        print(msg)
        print("Drone was forcefully Disarmed!")

    def change_mode(self, mode: str = 'GUIDED'):
        '''
        Autonomously changes the mode of the vehicle.

        :param mode: Type (str). Default -> 'GUIDED'.
                     It is the new mode for the vehicle.  ['GUIDED', 'AUTO', 'RTL', 'LAND', 'LOITER', etc.]
        :return: nothing
        '''
# Gets the possible modes of the vehicle and their id's, as a dictionary.
        mode_id = self.__conn.mode_mapping()[mode]
# self.__conn.set_mode(mode_id)
        self.__conn.mav.command_long_send(self.__conn.target_system,
                                          self.__conn.target_component,
                                          mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                          0,
                                          mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                          mode_id,
                                          0, 0, 0, 0, 0)
# Confirmation Message
        self.confirmation_message(f'Setting Mode to {mode}')
        
    def arm_and_takeoff(self, takeoff_altitude, verbose = True, skip_prearm_checks = False):
        """
        Arms the vehicle and takes off to the desired altitude, relative to the home location.

        :param takeoff_altitude:   This is in metres, relative to the home location.
        :param verbose:            Controls display of altitude update messages.
        :param skip_prearm_checks: This parameter controls whether to perform the pre-arm safety checks or to skip them.
        :return: nothing
        """
# Arming
        self.arm(skip_prearm_checks=skip_prearm_checks)
        
        if takeoff_altitude is None or takeoff_altitude == 0:
            return
# Takeoff
        print(f'Taking off to altitude of {takeoff_altitude} metres...')
        self.__conn.mav.command_long_send(
            self.__conn.target_system,
            self.__conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0, takeoff_altitude)
# Confirmation Message
        msg = self.__conn.recv_match(type="COMMAND_ACK", blocking=True)
        print(msg)
# Altitude Checking
        while True:
            msg = self.__conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if verbose:
                print(f"Relative Altitude: {msg.relative_alt / 1000}")
            if msg.relative_alt > takeoff_altitude * 1000 * 0.98:
                break
            time.sleep(1)
# Target reached message
        print(f"Reached target altitude of {takeoff_altitude} metres!")
    
    def land(self, disarm = True, verbose = True):
        """
        Lands the vehicle on the ground. Might disarm it, depending on the command given.

        :param disarm:  Controls whether to disarm the vehicle after landing it or not.
        :param verbose: Controls display of altitude update messages
        :return: nothing
        """
# Landing
        print(f'Landing...')
        self.__conn.mav.command_long_send(
            self.__conn.target_system,
            self.__conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0, 0, 0, 0, 0, 0, 0)
# Confirmation Message
        msg = self.__conn.recv_match(type="COMMAND_ACK", blocking=True)
        print(msg)
# Altitude Checking
        while True:
            msg = self.__conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if verbose:
                print(f"Relative Altitude: {msg.relative_alt / 1000}")
            if msg.relative_alt < 10:
                break
            time.sleep(1)
        print(f'Vehicle Landed!')
# Disarming
        if disarm:
            self.disarm()
            
    def goto_waypoint(self, target_lat, target_lon, target_alt = None, verbose = True):
        """
        Goes to a waypoint, based on the global latitude and longitude coordinates. The target altitude is relative to the home location.

        :param target_lat: Type (float). It is the unscaled latitude value of the target waypoint.
        :param target_lon: Type (float). It is the unscaled longitude value of the target waypoint.
        :param target_alt: Type (int). It is the target altitude, relative to the home location, in metres.
        :param verbose:    Controls display of location update messages.
        :return: nothing
        """
# Scaling of coordinates
        if target_alt is None:
            msg = self.__conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            target_alt = msg.relative_alt // 1000
        target_lat = int(target_lat * 1e7)
        target_lon = int(target_lon * 1e7)
# Command for location change
        self.__conn.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(0, self.__conn.target_system, self.__conn.target_component,
                                                                                            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                                                                            int(0b110111111000),
                                                                                            target_lat, target_lon, target_alt,
                                                                                            0, 0, 0, 0, 0, 0, 0, 0))
# Location checking
        print('Heading to waypoint!')
        start = False
        while True:
            msg = self.__conn.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)
            if verbose:
                print(f'Distance to waypoint: {msg.wp_dist}')
            if msg.wp_dist > 0:
                start = True
            elif msg.wp_dist == 0 and start:
                break
            time.sleep(3)
# Confirmation Message
        print("\nWaypoint reached!!!\n")
    
    def change_altitude(self, target_alt, verbose = True):
        """
        Changes altitude of the vehicle. Target altitude is given with respect to the home location.

        :param target_alt: Type (int). It is the target altitude, relative to the home location, in metres.
        :param verbose:    Controls display of altitude update messages.
        :return: nothing
        """
# Command for altitude change
        print(f'Changing altitude to {target_alt} metres')
        self.__conn.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(0, self.__conn.target_system, self.__conn.target_component,
                                                                                            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                                                                            int(0b110111111011),
                                                                                            0, 0, target_alt,
                                                                                            0, 0, 0, 0, 0, 0, 0, 0))
# Altitude checking
        start = False
        while True:
            msg = self.__conn.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)
            if verbose:
                print(f'Distance to target altitude: {msg.wp_dist} metres')
            if msg.wp_dist > 0:
                start = True
            elif msg.wp_dist == 0 and start:
                break
            time.sleep(3)
# Confirmation Message
        print(f"\nAltitude of {target_alt} metres reached!\n")

    def goto_ned_position(self, north_displacement, east_displacement, up_displacement, verbose = True):
        """
        Changes the position of the vehicle, based on North, East and Up directions. Units are in metres.

        :param north_displacement: Type (int). Distance to move in North direction in metres.
        :param east_displacement:  Type (int). Distance to move in East direction in metres.
        :param up_displacement:    Type (int). Distance to move up in metres. [Up -> Positive, Down -> Negative]
        :param verbose:            Controls display of location update messages.
        :return: nothing
        """
# Command for location change
        print(f"Moving {north_displacement} metres North, {east_displacement} metres East, {up_displacement} metres UP...")
        self.__conn.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, self.__conn.target_system, self.__conn.target_component,
                                                                                           mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                                                                                           int(0b110111111000),
                                                                                           north_displacement, east_displacement, up_displacement * -1,
                                                                                           0, 0, 0, 0, 0, 0, 0, 0))
# Location checking
        start = False
        while True:
            msg = self.__conn.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)
            if verbose:
                print(f'Distance to target altitude: {msg.wp_dist} metres')
            if msg.wp_dist > 0:
                start = True
            elif msg.wp_dist == 0 and start:
                break
            time.sleep(3)
# Confirmation Message
        print(f'New position reached!')
        print(f"Moved {north_displacement} metres North, {east_displacement} metres East, {up_displacement} metres UP!")

    def change_ned_altitude(self, target_alt, verbose = True):
        """
        Changes the altitude of the vehicle, relative to the current altitude of the vehicle.

        :param target_alt: Type (int). It is the distance the vehicle must move upwards in metres. [Up -> Positive, Down -> Negative]
        :param verbose:    Controls display of altitude update messages.
        :return: nothing
        """
# Command for altitude change
        print(f'Moving up {target_alt} metres') if target_alt > 0 else print(f'Moving down {target_alt} metres')
        self.__conn.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, self.__conn.target_system, self.__conn.target_component,
                                                                                           mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                                                                                           int(0b110111111011),
                                                                                           0, 0, target_alt * -1,
                                                                                           0, 0, 0, 0, 0, 0, 0, 0))
# Altitude checking
        start = False
        while True:
            msg = self.__conn.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)
            if verbose:
                print(f'Distance to target altitude: {msg.wp_dist} metres')
            if msg.wp_dist > 0:
                start = True
            elif msg.wp_dist == 0 and start:
                break
            time.sleep(3)
# Confirmation Message
        print(f'Moved up {target_alt} metres!') if target_alt > 0 else print(f'Moved down {target_alt} metres!')

    def change_speed(self, target_speed: float, ground_speed: bool = True):
        """
        Changes the speed of the vehicle. The speed type can be ground speed or air speed, depending on the argument passed.

        :param target_speed: Type (float). The target speed for the vehicle, in metres per second (m/s).
        :param ground_speed: Type (bool). Specifies which speed type to change.
                                          [True -> Ground-speed, False -> Air-speed]
        :return: nothing
        """
# Command for speed change
        speed_type = 1 if ground_speed else 0
        self.__conn.mav.command_long_send(self.__conn.target_system, self.__conn.target_component,
                                          mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                                          0,
                                          speed_type, target_speed, 0, 0, 0, 0, 0)
# Confirmation Message
        msg = self.__conn.recv_match(type="COMMAND_ACK", blocking=True)
        print(msg)
        print(f'Ground-speed of drone was changed to {target_speed}\n') if ground_speed else print(f'Air-speed of drone was changed to {target_speed}\n')
        time.sleep(5)

    def __set_global_speed(self, vx: float, vy: float, vz: float, duration: int):
        pass

    def set_servo(self, servo_number, servo_pwm):
        """
        Changes the pwm of the servo. Used for dropping of payload from the vehicle.

        :param servo_number: Servo ID to identify which servo's pwm to change.
                             If servo_n is the AUX port to set, servo_number is (servo_n + 8), offset by 8 MAIN outputs.
        :param servo_pwm:    Target pwm for the servo. This pulse-width is in microseconds.  [Between 1000 and 2000]
        :return: nothing
        """
# Command for setting the servo pwm
        self.__conn.mav.command_long_send(self.__conn.target_system, self.__conn.target_component,
                                          mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                                          0,
                                          servo_number, servo_pwm,
                                          0, 0, 0, 0, 0)
# Confirmation Message
        self.confirmation_message(task='Servo Command')

    def confirmation_message(self, task: str, timeout = None):
        """
        Mostly used internally. Confirms the successful completion of a command, or gives the state of the command or reason for failure.

        :param task:    Type (str). It specifies the name of the task/command.
        :param timeout: Type (int). It is the time in seconds to wait for the acknowledgement message.
        :return: nothing
        """
        msg = self.__conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=timeout)
        if msg.result == 0:
            print(f"{task} Successful!")
        elif msg.result == 4:
            print(f"{task} Unsuccessful!")
        elif msg.result == 3:
            print(f"{task} - This command is not supported.")
        elif msg.result == 2:
            print(f"{task} - Invalid command. -> Command is supported, but invalid parameters.")
        elif msg.result == 1:
            print(f"{task} - Temporarily rejected. -> Problem will be fixed by waiting, try again.")
        elif msg.result == 5:
            print(f"{task} - In progress...")
        elif msg.result == 9:
            print(f"{task} - Failed. -> Given mav_frame is not supported.")
        elif msg.result == 6:
            print(f"{task} - Command was cancelled.")
        else:
            print(f"{task} - Command failed. -> Result = {msg.result}")

    def get_attitude(self):
        print(f'Attitude of vehicle:')
        print(self.__conn.recv_match(type="AHRS2", blocking=True))
# print(self.__conn.recv_match(type="AHRS3", blocking=True))
        print(self.__conn.recv_match(type="AHRS", blocking=True), '\n')

    def get_speed(self):
        print(f'Speed of vehicle:')
        msg = self.__conn.recv_match("VFR_HUD", blocking=True)
        print(msg, '\n')


"""
Code starts here...
"""

#connection_string = 'udp:127.0.0.1:14550'

#master = Drone(connection_string=connection_string)

#mode_id = master.conn.mode_mapping()['GUIDED']
#master.conn.set_mode(mode_id)


"""
Code ends here
Exit
"""