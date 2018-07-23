import argparse
import time
import msgpack
import re
from enum import Enum, auto

import numpy as np

from planning_utils import calculate_waypoints, visualize_path
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection, goal_global_position=None):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.current_waypoint = np.array([0.0, 0.0, 0.0])
        self.waypoints = waypoints
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}
        self.initial_altitude = 10.0
        

        # initial state
        self.flight_state = States.MANUAL
        self.goal_global_position = goal_global_position

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)
    
    
    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.initial_altitude:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.current_waypoint[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()
                        
    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    print('D')
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path(self.waypoints)
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        print('SHIT')
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.initial_altitude)

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.next_waypoint = self.waypoints.pop(0)
        self.all_waypoints.append(self.next_waypoint)
        self.current_waypoint = self.next_waypoint[:-1]
        print('next waypoint', self.next_waypoint)
        self.cmd_position(self.next_waypoint[0], self.next_waypoint[1], self.next_waypoint[2], self.next_waypoint[3])
        return self.all_waypoints

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False
        print(time.time())

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self, waypoints):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")

        self.target_position = [waypoints[0], waypoints[1], waypoints[2]]
        # TODO: send waypoints to sim
        print(self.waypoints)
        print(self.target_position)

        return self.waypoints
        
    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()
        self.stop_log()


def read_home(filename):
    """
    Reads home (lat, lon) from the first line of the `file`.
    """
    with open(filename) as f:
        first_line = f.readline()
    match = re.match(r'^lat0 (.*), lon0 (.*)$', first_line)
    if match:
        lat = match.group(1)
        lon = match.group(2)
    return np.fromstring(f'{lat},{lon}', dtype='Float64', sep=',')


if __name__ == "__main__":
    
    if not sys.warnoptions:
        import warnings
        warnings.simplefilter("ignore")
      
    data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
    planning_start_time = time.time()
    colliders_file = 'colliders.csv'
    lat0, lon0 = read_home(colliders_file)
    print(f'Home lat : {lat0}, lon : {lon0}')
    global_home = [lon0, lat0, 0]
    
    drone_altitude = 8
    safety_distance = 2.0
    
    print(f'Global Home     => [lon, lat, alt]     : {global_home}')
    global_position = np.array([-122.3974512, 37.7924799, 0.147])
    print(f'Global Position => [lon, lat, alt]     : {global_position}')
    local_position = global_to_local(global_position, global_home)
    print(f'Local Position  => [north, east, down] : {local_position}')
    
    goal_global_position = [-122.400666, 37.791330, -1.0*drone_altitude]
    print(f'Goal_Global Position => [lon, lat, alt]     : {goal_global_position}')
    goal_local_position = global_to_local(goal_global_position, global_home)
    print(f'Goal_Local Position  => [north, east, down] : {goal_local_position}')

    nPoints = 2000
    waypoints, unpruned_path, pruned, graph, grid = calculate_waypoints(global_home, goal_global_position, 
                                                                        global_home, data, drone_altitude, safety_distance, nPoints, 10)
    #waypoints = waypoints[0]
    planning_end_time = time.time()
    print('Route planning time: ', planning_end_time-planning_start_time)
    
    visualize_path(data, local_position, goal_local_position, grid, graph, waypoints)
    
    '''
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port))
    drone = MotionPlanning(conn, waypoints)
    time.sleep(1)
    drone.start()
    '''
