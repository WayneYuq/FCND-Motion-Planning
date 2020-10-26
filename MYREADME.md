# FCND - 3D Motion Planning

### 1: Explain what's going on in `motion_planning.py` and `planning_utils.py`

- The main class `MotionPlanning` in `motion_planning.py` is response to the events from the drone such as velocity, state or position.
- Each events has a callback function so it's _Event Driven Programming_.
- When the instance drone start, it arm and take a path planning, if found a path, then takeoff and follow the waypoints.
- There's some different from `backyard_flyer_solution.py`, it need to planning a path itself and send the waypoints to
  the simulator for visualization.
- In `plan_path` method first set the home position depend on the first line of csv file `lat0, lon0`, create grid or
    graph representation of the map, then search the pathm, I use graph instead of grid, and prune the path.
- `planning_utils.py` has a set of utils functions such as create a grid map, and valid actions from current position(used
  in grid search algorithm), and A* search funciton. I also added some other utils it.
  
  
### 2: Set my current local position

- Read `lat0, lon0` from colliders.csv to set global home position.
- Then retrieve current global position and use `global_to_local` to convert to local postion.

### 3: Set graph start position from local position

- I use graph representation instead of grid.
- Use a binary search to determine if the line between two points is collide with obstacles.
- Set start position just current local position.
- Build a `Sampler` class to extract sample points and also a random goal position.
- Extract polygons from the csv file and `KDTree` to search from neighbors of each points,
    use polygon can determine the collide easily.
    
### 4: Set grid goal position from geodetic coords

- Set the goal in `sample_goal` method in `Sampler` class.
- If the lat and lon given by parameters is collide with obstacles, set it to the closest point.

### 5: Replace A* to graph search

- All the part remain the same but for actions from current node.
- Now the *action* is next connect node.

### 6: Cull waypoints

- Use collinearity check to cull waypoints.

