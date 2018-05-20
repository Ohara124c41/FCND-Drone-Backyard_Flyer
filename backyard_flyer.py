import argparse
import time
import visdom
import csv
import datetime
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

class MyDrone(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        # default opens up to http://localhost:8097
        self.v = visdom.Visdom()
        assert self.v.check_connection()

        # Plot NE
        ne = np.array([self.local_position[0], self.local_position[1]]).reshape(1, -1)
        self.ne_plot = self.v.scatter(ne, opts=dict(
            title="Local position (north, east)", 
            xlabel='North', 
            ylabel='East'
        ))

        # Plot D
        d = np.array([self.local_position[2]])
        self.t = 0
        self.d_plot = self.v.line(d, X=np.array([self.t]), opts=dict(
            title="Altitude (meters)", 
            xlabel='Timestep', 
            ylabel='Down'
        ))

        self.register_callback(MsgID.LOCAL_POSITION, self.update_ne_plot)
        self.register_callback(MsgID.LOCAL_POSITION, self.update_d_plot)

    def update_ne_plot(self):
        ne = np.array([self.local_position[0], self.local_position[1]]).reshape(1, -1)
        self.v.scatter(ne, win=self.ne_plot, update='append')

    def update_d_plot(self):
        d = np.array([self.local_position[2]])
        # update timestep
        self.t += 1
        self.v.line(d, X=np.array([self.t]), win=self.d_plot, update='append')

class BackyardFlyer(Drone):

    def __init__(self, connection, wpError_distance = .5):
        self.run_preffix = f'{datetime.datetime.now():%Y-%m-%d-%H-%M-%S}'
        telemetry = self.run_preffix + ' Telemetry Log'        
        super().__init__(connection, tlog_name=telemetry)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}
        self.wpError_distance = wpError_distance

        # initial state
        self.flight_state = States.MANUAL

        # Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)


    def local_position_callback(self):

        # This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                if len(self.all_waypoints) == 0:
                    self.all_waypoints = self.calculate_box()
                self.waypoint_transition()
                self.flight_state = States.WAYPOINT
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < self.wpError_distance:
                if len(self.all_waypoints) > 0: 
                    self.waypoint_transition()
                    self.flight_state = States.WAYPOINT
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()
                        self.flight_state = States.LANDING
        
    def velocity_callback(self):
        
        #This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()
                    self.flight_state = States.DISARMING
        
    def state_callback(self):
        """
        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """
        if self.in_mission: 
            if self.flight_state == States.MANUAL:
                self.arming_transition()
                self.flight_state = States.ARMING
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.takeoff_transition()
                    self.flight_state = States.TAKEOFF
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()
                    self.flight_state = States.MANUAL
        
    def calculate_box(self):
        
        # 1. Return waypoints to fly a box
        print('RETURNING WAYPOINTs')
        Waypoints = [[20.0, 0.0, 3.0], [20.0, 20.0, 3.0], [0.0, 20.0, 3.0], [0.0, 0.0, 3.0]]
        return Waypoints
        pass

    def arming_transition(self):
        """        
        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        print('ARMING state transition')
        self.take_control()
        self.arm()
        self.set_home_position(self.global_position[0], self.global_position[1],
                               self.global_position[2])  
    def takeoff_transition(self):
        """        
        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        altitudeGoal = 3.0
        self.target_position[2] = altitudeGoal
        self.takeoff(altitudeGoal)

        print("TAKEOFF state transition")

    def waypoint_transition(self):
        """    
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        nextWaypoint = self.all_waypoints.pop(0) 
        self.target_position = nextWaypoint 
        print('WAYPOINT acquired', nextWaypoint)
        self.cmd_position(nextWaypoint[0], nextWaypoint[1], nextWaypoint[2], 0.0)

        print("WAYPOINT state transition")

    def landing_transition(self):
        """        
        1. Command the drone to land
        2. Transition to the LANDING state
        """
        print('LANDING state transition')
        self.land()

    def disarming_transition(self):
        """        
        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        print('DISARMING state transition')
        self.disarm()
        self.release_control()

    def manual_transition(self):
        """        
        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("MANUAL state transition")
        originDistance = 20*np.linalg.norm(self.local_position) # vector norm from numpy
        print(f'Distance to origin : {originDistance:5.3f} cm')

        self.release_control()
        self.stop()
        self.in_mission = False

    def start(self):
        """        
        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("CREATING log file")
        self.start_log("Logs", "NavLog.txt")
        print("STARTING connection")
        self.connection.start()
        while self.in_mission:
           pass
        print("CLOSING log file")
        self.stop_log()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=True, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
