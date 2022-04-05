# Advanced waypoint package

The advanced waypoint package consists of the planner_node and the planner.

This package plans the trajectory of the drone when the drone is performing  waypoint-following and not vector-following.
See controller_pkg/src/controller_node for the decision logic between vector-following or waypoint-following.


- The planner_node instantiates the planner and sets the maximum speed and acceleration of the drone.
- The planner receives the number of waypoints and the waypoints from the waypoint publisher.
- snap trajectory is calculated using the mav_trajectory_generation package from Lab 6.
- The trajectory and the markers for it are then published.


## Input
- numberOfWaypoints message:
  number of waypoints for which the trajectory should be calculated
- waypoint message:
  next waypoint for the trajectory
- current_state:
  current state of the drone

## Output
- trajectory message:
  contains the calculated trajectory
- trajectory_markers
  markers to visualise the trajectory in rviz


(Maximilian and Samuel)
