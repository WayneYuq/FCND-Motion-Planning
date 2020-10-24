# FCND - 3D Motion Planning

### 1: Explain what's going on in `motion_planning.py` and `planning_utils.py`

- The main class `MotionPlanning` in `motion_planning.py` is response to the events from the drone such as velocity, state or position.
- Each events has a callback function so it's _Event Driven Programming_.
- When the instance drone start, it arm and take a path planning, if found a path, then takeoff and follow the waypoints.
- There's some different from `backyard_flyer_solution.py`, it need to planning a path itself and send the waypoints to
  the simulator for visualization.
- `planning_utils.py` has a set of utils functions such as create a grid map, and valid actions from current position(used
  in grid search algorithm), and A* search funciton. I also added some other utils it.
  
 