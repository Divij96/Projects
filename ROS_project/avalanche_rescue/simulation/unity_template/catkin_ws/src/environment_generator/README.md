# environment_generator package

The environment_generator package consists of the environment_generator_node and a configuration.

The environment_generator_node loads the parameters from the src/environment_generator/config/environment_params.yaml. It then defines the search area and publishes the rviz markers and coordinates of the corners.


## Input
- current_state message:
  current state of the drone
- avalanche message: contains the corners for the search area

## Output
- avalanche message for the waypoint_publisher_node
- markers for rviz

~ Nicholas
