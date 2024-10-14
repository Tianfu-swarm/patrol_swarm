# Topology_mapping

## Description

## Usage
### Publish message
To publish the `patrolArea` message, use the following command:

```bash
ros2 topic pub /patrolArea topology_map_creator/msg/Area2D "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, points: [{x: 1.0, y: 2.0, z: 0.0}, {x: 3.0, y: 4.0, z: 0.0}]}}"
```

To publish the `obstacleArea` message, adjust the command as necessary (not specified here, but can follow a similar format).
