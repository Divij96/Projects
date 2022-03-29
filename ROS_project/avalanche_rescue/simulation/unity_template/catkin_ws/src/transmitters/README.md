# Transmitter Node

This node creates random positions of the transmitters on the search plane with a relative offset to the search area boundarys and sets their orientation. Therefore the avalanche corners set in `~/environment_generator/config/environment_params.yaml` are used.
The number of created transmitters depends on the preset number of beacons. The information about the position, orientation and transmitter id are published to the `/transmitter/pose`topic in form of a costum message. In addition the pose information is also published to rviz via the `transmitter/marker` topic in form of a `MarkerArray` message type. (by Benedikt Fischhaber)

## Input 
- Parameters from parameter server

## Output 
- `transmitter/pose` as `transmitters::transmitter_array`
- `transmitter/marker` as `visualization_msgs::MarkerArray`




