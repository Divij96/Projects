# unity_state.cpp
Mainly based on the script given in different labs. Only added a marker message for the drone visualization in rviz. Therefore the drone mesh given in Lab2 is used. (by Benedikt Fischhaber)

**Publisher** 
- `/current_state_marker` as `visualization_msgs::Marker`

# vector_follower.cpp
Based on the traj_publisher.cpp.

This node is decides the movement of the drone when the drone is performing vector-following and not waypoint-following.
See controller_pkg/src/controller_node for the decision logic between vector-following or waypoint-following.

- Uses the flux vector supplied by the receiverNode to calculate the desired movement for vector following.

- Uses the distance to the beacon to calculate the desired speed of the drone towards the beacon.

- Calculates the z-Position of the drone relative to the plane in which the avalanche corners lie and
uses this information to make sure that the drone keeps a distance to the snow while vector-following.

- Sends out the desired_state to the controller while vector-following

(by Samuel Zeitler)

## w_to_unity.cpp ##

unedited.



