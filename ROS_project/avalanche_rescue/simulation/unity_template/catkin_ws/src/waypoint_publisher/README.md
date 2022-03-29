# waypoint_publisher package

The waypoint_publisher package only consists of the waypoint_publisher_node.

The waypoint_publisher_node receives the avalanche message from the environment generator with the corners of the search area.
If the search pattern was not generated yet, a new search pattern is calculated using the sensor range and the stepSize (distance between waypoints) as parameters.
The final search pattern is saved as a list of waypoints. After the generation of the waypoints the number of waypoints is published for the planner.

If the unity environment gets started, the waypoint_publisher publishes the waypoints in the list one by one.
These waypoints are then used by the planner in the advanced_waypoint_pkg to generate a trajectory.

After publishing the number of waypoints and the search pattern waypoints, this node stays idle for the rest of the simulation.

Somewhere in this node the easter egg is implemented ;)

## Input
- current_state message:
  current state of the drone
- avalanche message: contains the corners for the search area

## Output
- numberOfWaypoints message: number of waypoints for the trajectory generation
- waypoint message: the next waypoint in the search pattern which should be included in the trajectory

Written by Nicholas and Maximilian
