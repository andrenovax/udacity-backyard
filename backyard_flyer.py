import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5

CONFIG = {
  "WAYPOINT_TRANSITION_DISTANCE": 0.2,
  "SAFE_LANDING_DISTANCE": 0.05,
  "TARGET_ALTITUDE": 3.0
}

class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def is_safe_to_disarm(self):
        return self.local_position[2] < CONFIG["SAFE_LANDING_DISTANCE"]

    def has_reached_xy_target(self, target_position):
        distance = np.linalg.norm(target_position[0:2] - self.local_position[0:2])
        return distance < CONFIG["WAYPOINT_TRANSITION_DISTANCE"]

    def has_reached_z_target(self, target_position):
        distance = abs(target_position[2] + self.local_position[2])
        return distance < CONFIG["WAYPOINT_TRANSITION_DISTANCE"]

    def local_position_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """
        match self.flight_state:
            case States.TAKEOFF:
                if self.has_reached_z_target(self.target_position):
                    self.all_waypoints = self.calculate_box()
                    self.waypoint_transition()
            case States.WAYPOINT:
                if self.has_reached_xy_target(self.target_position):
                    if len(self.all_waypoints) > 0:
                        self.waypoint_transition()
                    else:
                        self.landing_transition()
            case States.LANDING:
                if self.has_reached_z_target(self.global_home) and self.is_safe_to_disarm():
                    self.disarming_transition()

    def velocity_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
        pass

    def state_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """
        if self.in_mission:
            match self.flight_state:
                case States.MANUAL:
                    self.arming_transition()
                case States.ARMING:
                    if self.armed:
                        self.takeoff_transition()
                case States.DISARMING:
                    if not self.armed and self.guided:
                        self.manual_transition()

    def calculate_box(self):
        """TODO: Fill out this method
        
        1. Return waypoints to fly a box
        """
        waypoints = [
            [100.0, 0.0, CONFIG["TARGET_ALTITUDE"]],
            [100.0, 100.0, CONFIG["TARGET_ALTITUDE"]],
            [0.0, 100.0, CONFIG["TARGET_ALTITUDE"]],
            [0.0, 0.0, CONFIG["TARGET_ALTITUDE"]],
        ]
        return waypoints

    def arming_transition(self):
        """TODO: Fill out this method
        
        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        print("arming transition")

        self.take_control()
        self.arm()
        lat, lon, alt = self.global_position
        self.set_home_position(lat, lon, alt)
        self.flight_state = States.ARMING

    def takeoff_transition(self):
        """TODO: Fill out this method
        
        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        print("takeoff transition")

        self.target_position[2] = CONFIG["TARGET_ALTITUDE"]
        self.takeoff(CONFIG["TARGET_ALTITUDE"])
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        """TODO: Fill out this method
    
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        print("waypoint transition")
        self.target_position = self.all_waypoints.pop(0)
        north, east, down = self.target_position
        self.cmd_position(north, east, down, 0)
        self.flight_state = States.WAYPOINT

    def landing_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to land
        2. Transition to the LANDING state
        """
        print("landing transition")

        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        print("disarm transition")

        self.disarm()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        """This method is provided
        
        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("manual transition")

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        """This method is provided
        
        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
