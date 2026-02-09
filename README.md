# map_only_launch

A minimal ROS 2 package for launching a static map server and publishing mock TurtleBot4 data for testing and development purposes.

## Overview

This package provides:
- **Static Map Server**: Launches the Nav2 map server to publish a warehouse map on the `/map` topic
- **Mock Data Publisher**: A standalone Python node for publishing sample navigation goals, odometry, velocity commands, and rule outputs to test TurtleBot4-based applications without hardware

## Features

### Map Server Launch
- Automatically launches the Nav2 map server with a preconfigured warehouse map
- Uses lifecycle manager for automatic activation
- Publishes to the standard `/map` topic

### Pose Publisher Node
The `pose_publisher` node creates and publishes mock data to multiple TurtleBot4 topics:

- **Goal Poses** (`/gary/goal_pose`): Publishes sample navigation goals as `PoseStamped` messages
- **Rule Outputs** (`/gary/rule_output`): Publishes corresponding navigation rules in JSON format
- **Odometry** (`/odom`): Publishes mock robot position data
- **Velocity Commands** (`/cmd_vel`): Publishes velocity commands for robot movement control
- **Dock Status** (`/dock_status`): Publishes robot docking status (currently commented out)

## Repository Structure

```
map_only_launch/
├── launch/
│   └── map_server_launch.py       # Launch file for map server
├── map_only_launch/
│   ├── __init__.py
│   └── pose_publisher.py          # Mock data publisher node
├── resource/
│   └── map_only_launch            # Package resource marker
├── test/                          # Test directory
├── warehouse_map.json             # Map data (JSON format, ~2.6 MB)
├── warehouse_map.png              # Map visualization
├── package.xml                    # ROS 2 package manifest
├── setup.py                       # Python package setup
├── setup.cfg                      # Python package configuration
└── README.md                      # This file
```

## Dependencies

### Runtime Dependencies
- ROS 2 Humble (or compatible)
- `nav2_map_server` - For serving static maps
- `nav2_lifecycle_manager` - For managing map server lifecycle
- `irobot_create_msgs` - TurtleBot4/iRobot Create message definitions
- Standard ROS 2 message packages:
  - `geometry_msgs`
  - `nav_msgs`
  - `std_msgs`

### Build Dependencies
- `ament_python`
- `launch`
- `launch_ros`

### Test Dependencies (optional)
- `ament_copyright`
- `ament_flake8`
- `ament_pep257`
- `pytest`

## Installation

1. **Clone the repository** into your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone <repository-url> map_only_launch
   ```

2. **Install dependencies**:
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the package**:
   ```bash
   colcon build --packages-select map_only_launch
   ```

4. **Source the workspace**:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## Usage

### Launch the Map Server

To start the map server and publish the warehouse map:

```bash
ros2 launch map_only_launch map_server_launch.py
```

This will:
- Start the `nav2_map_server` node with the warehouse map configuration
- Automatically activate the map server using `nav2_lifecycle_manager`
- Publish the map on the `/map` topic

### Run the Mock Data Publisher

To publish sample navigation goals and odometry data:

```bash
ros2 run map_only_launch pose_publisher
```

This will:
1. Publish 3 sample goal poses at intervals (default: 3 seconds between each)
2. Publish corresponding rule outputs for each goal
3. Publish a sequence of odometry positions along the path
4. Publish a stop command
5. Keep publishers active until interrupted (Ctrl+C)

### Verify Topics

Check that topics are being published:

```bash
# List all topics
ros2 topic list

# Echo the map topic
ros2 topic echo /map

# Echo goal poses
ros2 topic echo /gary/goal_pose

# Echo odometry
ros2 topic echo /odom
```

## Configuration

### Map Configuration

The map server is configured to use the TurtleBot4 warehouse map:
- **Map YAML**: `/opt/ros/humble/share/turtlebot4_navigation/maps/warehouse.yaml`
- **Map Files**: Included in the repository as `warehouse_map.json` and `warehouse_map.png`

To use a different map, modify the `yaml_filename` parameter in `launch/map_server_launch.py`:

```python
parameters=[{
    'yaml_filename': '/path/to/your/map.yaml'
}]
```

### Mock Data Configuration

The `pose_publisher` node publishes three sample goals by default:
- **Goal 1**: (1.0, 1.0) - Intermediate waypoint
- **Goal 2**: (2.0, 2.0) - Intermediate waypoint
- **Goal 3**: (3.0, 3.0) - Global goal

You can modify these goals in `map_only_launch/pose_publisher.py` in the `publish_sample_goals()` method.

## Published Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/map` | `nav_msgs/OccupancyGrid` | Static occupancy grid map |
| `/gary/goal_pose` | `geometry_msgs/PoseStamped` | Navigation goal poses |
| `/gary/rule_output` | `std_msgs/String` | Navigation rules (JSON format) |
| `/odom` | `nav_msgs/Odometry` | Robot odometry data |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
| `/dock_status` | `irobot_create_msgs/DockStatus` | Docking status (currently disabled) |

## QoS Configuration

The mock publisher uses the following QoS profile for reliable communication:
- **Reliability**: `RELIABLE`
- **History**: `KEEP_LAST` (depth: 10)
- **Durability**: `TRANSIENT_LOCAL`

This ensures that late-joining subscribers receive the most recent messages.

## Development

### Running Tests

```bash
colcon test --packages-select map_only_launch
colcon test-result --verbose
```

### Code Style

This package follows ROS 2 Python coding standards:
- PEP 257 (docstring conventions)
- PEP 8 (style guide)
- Checked with `flake8` and `ament_copyright`

## Troubleshooting

### Map Not Publishing
- Ensure `nav2_map_server` is installed: `ros2 pkg list | grep nav2_map_server`
- Check if the map YAML file exists at the specified path
- Verify the lifecycle manager is transitioning the map server to active state

### Topics Not Visible
- Make sure to source the workspace: `source ~/ros2_ws/install/setup.bash`
- Check if the node is running: `ros2 node list`
- Verify ROS_DOMAIN_ID matches across terminals

### Import Errors
- Install missing dependencies: `rosdep install --from-paths src --ignore-src -r -y`
- Ensure `irobot_create_msgs` is installed for TurtleBot4 message types

## License

Apache License 2.0

## Maintainer

**Saadhvi**  
Email: uswup@student.kit.edu

## Contributing

This is a minimal development package. For contributions or issues, please contact the maintainer.

## Acknowledgments

- Uses the TurtleBot4 warehouse map from the `turtlebot4_navigation` package
- Built with ROS 2 and Nav2 navigation stack
