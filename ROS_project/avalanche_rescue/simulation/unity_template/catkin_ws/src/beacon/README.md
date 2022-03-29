# Beacon package

The Beacon package consists of the receiver_node and the visualisation markers for visualisation in rviz.

-This package electromagnetic flux lines based on the orientation and position of the beacons received from the transmitter node and
the orientation and position of the drone and sends the flux line vector and the estimated distance between the transmiter and drone.

-After all the beacons are identified, it generates a final report which includes estimated positions and true position of the beacon transmiters.

-It generates the vector visualisation of the estimated position of the beacon and vector pointing to the identified beacon.


## Input
- position and orientation of the beacon transmitters.

## Output
- beacon id.
  estimated distance to the beacon(in cm).
  electromagnetic flux line vector to the identified beacon.

Created by: Divij Grover, Benedikt Fischhaber, Samuel Zeitler
