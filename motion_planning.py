import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import a_star, a_star_graph, heuristic, create_grid, prune_path
from sampling import Sampler
from graph import Graph
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local

"""
Udacity Instructor Reviews:
At the moment there is some mismatch between the colliders map and actual buildings in the scene. 
To ensure success build in a 5+ m safety margin around obstacles. Try some different goal locations. 
Also try starting from a different point in the city. Your reviewer will also try some random 
locations so be sure to test your solution! There is no firm constraint or requirement on how 
accurately you land exactly on the goal location. Just so long as your planner functions as expected.
"""


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
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
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2],
                          self.target_position[3])

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

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # Read lat0, lon0 from colliders.csv into floating point values
        # I just use `open()` method and `f.readline()` to read the first line,
        # which line it read depend on the times it calls, I just call it once so it's the first line,
        # then slice out the number
        with open('colliders.csv', 'r') as f:
            first_line_data = f.readline().rstrip().split(', ')
            lat0 = float(first_line_data[0][5:])
            lon0 = float(first_line_data[1][5:])

        # Set home position
        self.set_home_position(lon0, lat0, 0)

        # Retrieve current global position
        gp = self.global_position  # (lon, lat, up)

        # Convert to current local position
        lp = global_to_local(gp, self.global_home)  # (north, east, down)

        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))

        # Read in obstacle map
        t0 = time.time()
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        print("Took {0} seconds to load csv file".format(time.time() - t0))

        # Convert start position to current position
        start = (lp[0], lp[1], TARGET_ALTITUDE)

        t0 = time.time()
        sampler = Sampler(data, self.global_home)
        print("Took {0} seconds to initialize Sampler class".format(time.time() - t0))
        t0 = time.time()
        nodes = sampler.sample(30)

        # Set goal as some arbitrary position on the map
        goal = sampler.sample_goal(nodes, 37.793, -122.4)

        print('Local Start and Goal: ', start, goal)

        nodes.append(start)
        nodes.append(goal)
        print("Took {0} seconds to create {1} sample nodes.".format(time.time() - t0,  len(nodes)))

        t0 = time.time()
        g = Graph(sampler.polygons, sampler.polygon_center_tree, sampler.max_poly_xy)
        g.create_graph(nodes, 10)
        print("Took {0} seconds to build graph".format(time.time() - t0))
        print("Number of edges:", len(g.edges))

        t0 = time.time()
        # Convert grid search to graph
        path, _ = a_star_graph(g.graph, heuristic, start, goal)
        print("Took {0} seconds to search path".format(time.time() - t0))

        # Prune path to minimize number of waypoints
        t0 = time.time()
        path = prune_path(path)
        print("Took {0} seconds to prune path".format(time.time() - t0))

        # Convert path to waypoints
        waypoints = [[int(p[0]), int(p[1]), TARGET_ALTITUDE, 0]for p in path]
        self.waypoints = waypoints

        # Send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=3000)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
