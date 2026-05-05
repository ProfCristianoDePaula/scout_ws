# Scout Control Package

This package contains ROS2 nodes for controlling the Scout Mini robot.

## Nodes

### obstacle_avoidance

A reactive obstacle avoidance node for autonomous navigation in maze environments.

#### Description
The `obstacle_avoidance` node implements a simple reactive controller that:
- Processes LaserScan data to detect obstacles
- Moves forward until an obstacle is detected within 0.9 meters in the frontal sector (±10°)
- Stops and evaluates left and right spaces using extreme lateral sectors (75°-90° and -90°--75°)
- Turns 90° to the side with more space (left if left > right, right otherwise)
- Performs a fixed 10-second turn for consistency
- Moves forward for 2 seconds after turning to stabilize
- Repeats until near the goal (within 2 meters)
- Performs a final right turn and approaches the goal

#### Parameters
- **Linear Speed**: 0.4 m/s
- **Angular Speed**: 1.5 rad/s
- **Safe Distance**: 0.9 meters (frontal)
- **Turn Duration**: 10 seconds
- **Post-Turn Forward**: 2 seconds
- **Goal Threshold**: 1.0 meter
- **Final Approach Distance**: 2.0 meters from goal

#### Topics
- **Subscribers**:
  - `/scan` (sensor_msgs/LaserScan): Laser scan data
  - `/odom` (nav_msgs/Odometry): Odometry for position and yaw
- **Publishers**:
  - `/cmd_vel` (geometry_msgs/Twist): Velocity commands

#### States
1. **FORWARD**: Moving forward
2. **STOP**: Stopped, deciding turn direction
3. **TURN**: Turning in place for 10 seconds
4. **POST_TURN_FORWARD**: Moving forward after turn
5. **FINAL_TURN**: Preparing final turn near goal
6. **FINAL_MOVE**: Final turn
7. **GOAL**: Approaching goal

#### Usage
```bash
# Terminal 1: Launch simulation
ros2 launch scout_sim scout_sim_maze.launch.py

# Terminal 2: Run obstacle avoidance
ros2 run scout_control obstacle_avoidance
```

#### Logs
- Periodic status logs every second: State, Position, Yaw, Distances
- Turn decisions and completions

#### Notes
- Designed for Scout Mini in maze environment from (-8,8) to (8,-7)
- Uses fixed-time turns to avoid odometry drift issues
- Frontal sector narrowed to ±10° to avoid lateral wall interference
- Lateral sectors focused on extreme sides for accurate space assessment

### Other Nodes
- `move_square`: Moves robot in a square pattern
- `move_circle`: Moves robot in a circle
- `sensor_reader`: Reads and logs sensor data
- `obstacle_stop`: Basic obstacle stopping
- `challenge2_navigator`: Navigator for challenge 2
- `maze_obstacle_avoidance`: Previous maze avoidance implementation

## Building
```bash
cd ~/scout_ws
colcon build --packages-select scout_control
```

## Dependencies
- ROS2 Jazzy
- scout_sim package for simulation
- Standard ROS2 message types