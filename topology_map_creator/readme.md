# Topology_mapping

## Description

## Usage
### Publish message
To publish the `patrolArea` message, use the following command:

```bash
ros2 topic pub /mapArea topology_map_creator/msg/Area2D "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, points: [{x: 0.0, y: 0.0, z: 0.0}, {x: 1.0, y: 0.0, z: 0.0}, {x: 1.0, y: 1.0, z: 0.0}, {x: 0.0, y: 1.0, z: 0.0}]}"
```
To publish the `obstacleArea` message, use the following command:

#### 1 obstacle
```bash
ros2 topic pub /obstacleArea topology_map_creator/msg/Obstacle "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, obstacles: [{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, points: [{x: 0.0, y: 0.0, z: 0.0}, {x: 0.5, y: 0.0, z: 0.0}, {x: 0.5, y: 0.5, z: 0.0}, {x: 0.0, y: 0.5, z: 0.0}]}]}"
```
#### 2 obstacle
```bash
ros2 topic pub /obstacleArea topology_map_creator/msg/Obstacle "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, 
obstacles: [
    {header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, points: [{x: 0.0, y: 0.0, z: 0.0}, {x: 0.5, y: 0.0, z: 0.0}, {x: 0.5, y: 0.5, z: 0.0}, {x: 0.0, y: 0.5, z: 0.0}]},
    {header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, points: [{x: 0.5, y: 0.5, z: 0.0}, {x: 1.0, y: 0.5, z: 0.0}, {x: 1.0, y: 1.0, z: 0.0}, {x: 0.5, y: 1.0, z: 0.0}]}
]}"

```

### start
To run the code:
```bash
ros2 launch topology_map_creator run.launch.py
```