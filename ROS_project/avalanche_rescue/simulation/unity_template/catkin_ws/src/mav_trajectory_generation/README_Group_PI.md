# mav_trajectory_generation package

This package was slightly modified to adapt the sampler for the search.
The modified file is the trajectory_sampler_node in the mav_trajectory_generation_ros package.

To stop the sampler during the vector following, a subscriber to the receiver/signal was added to the node.
If the receiver detects a beacon, the distance sent in the receiver/signal message is greater than -1.
Then the planner is stoppped from publishing messages to the controller.
Doing so, the trajectory publishing is not continued during the vector following and the drone can continue
its flight at the point where it left the predefined trajectory after the search procedure.

If this mechanism was not implemented, the sampler keeps on publishing the trajectory in the background during the search of the
beacon. If the beacon is found and the drone has to continue with the search pattern, the next command from the sampler is not
the next one in the expected pattern, but a position further away. Like this, some beacons could be missed.

All other parts of this package are in original condition.

Changes made by Nicholas and Maximilian
