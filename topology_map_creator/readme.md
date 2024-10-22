# Topology_mapping

## Description

## Usage
### Publish message
To publish the `patrolArea` message, use the following command:

```bash
ros2 topic pub /mapArea topology_map_creator/msg/Area2D "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, points: [{x: 0.0, y: 0.0, z: 0.0}, {x: 1.0, y: 0.0, z: 0.0}, {x: 0.5, y: 0.5, z: 0.0}]}"
```
To publish the `obstacleArea` message, adjust the command as necessary (not specified here, but can follow a similar format).

### start
To run the code:
```bash
ros2 launch topology_map_creator run.launch.py
```