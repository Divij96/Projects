## Controller Node ##

based on the controller_node.cpp from Lab_6

The controller_node:
- subscribes to the receiverNode and receives the distance of the drone to the beacon
- subscribes to the vector_follower and receives the desired position
of the drone when vector-following
- subscribes to the planner_node and receives the desired position
of the drone when waypoint-following
- subscribes to the current_state
- publishes the propeller speeds

The core logic in the controller is to decide wether to use vector-following or waypoint-following.
It does this by checking if the drone is close enough to a beacon.

Both the vector_follower and the planner_node supply a desired position which the controller
from Lab_6 converts into propeller speeds.

(Samuel Zeitler)



