# Skratch Sensor Package

This package provides sensor configuration and integration for the Skratch robot, including **dual 2D LiDAR scan merging** for robots equipped with front and rear laser scanners.

---

## Package Overview

The `skratch_sensor` package handles sensor data processing, specifically:

1. **Dual Laser Merger Node**
   - Uses the `dual_laser_merger` package
   - Merges multiple `sensor_msgs/LaserScan` topics into one unified scan
   - Automatically launched with the Gazebo simulation

2. **Laser Scan Relays**
   - Republishes front and rear laser scans into a common topic
   - Uses `topic_tools/relay`

> [!NOTE]
> The dual laser merger is automatically integrated into the Gazebo simulation launch. You don't need to launch it separately when running `ros2 launch skratch_gazebo gazebo.launch.py`.

---

## Package Structure

```
skratch_sensor/
├── config/
│   └── dual_laser_merger_params.yaml
├── docs/
│   └── (documentation files)
├── launch/
│   ├── dual_laser_merger.launch.py
│   └── laser_scan_relay.launch.py
└── package.xml
```

---

## Launch Files

### Dual Laser Merger Launch

This launch file starts the laser merger node and loads parameters from a YAML configuration file.

**Node launched:**
- Package: `dual_laser_merger`
- Executable: `dual_laser_merger_node`
- Node name: `dual_alaser_merger`

**Parameter file location:**
`skratch_sensor/config/dual_laser_merger_params.yaml`

This file defines:
- Input laser scan topics
- Output topic
- TF frames
- Angular and range limits

---

### Laser Scan Relay Launch

This launch file republishes individual laser scans into a shared topic.

**Relays:**
- `/front_scan` → `/scan_combined`
- `/rear_scan` → `/scan_combined`

This allows downstream nodes to subscribe to a **single laser scan topic**.

---

## Topic Flow

```
/front_scan ─┐
             ├──► /scan_combined ───► dual_laser_merger_node ───► /merged_scan
/rear_scan ─┘
```

---

## Topics

| Topic Name       | Message Type             | Description                   |
|------------------|--------------------------|-------------------------------|
| `/front_scan`    | `sensor_msgs/LaserScan`  | Front LiDAR scan              |
| `/rear_scan`     | `sensor_msgs/LaserScan`  | Rear LiDAR scan               |
| `/scan_combined` | `sensor_msgs/LaserScan`  | Combined laser scan output    |

---

## Usage

### Automatic Launch (with Gazebo)

The dual laser merger is **automatically launched** when you start the Gazebo simulation:

```bash
ros2 launch skratch_gazebo gazebo.launch.py
```

### Manual Launch (Standalone)

If you need to launch the sensor processing separately:

#### Build the workspace
```bash
cd ~/ros_robots/skratch_ws
colcon build --packages-select skratch_sensor
source install/setup.bash
```

#### Launch laser scan relays
```bash
ros2 launch skratch_sensor laser_scan_relay.launch.py
```

#### Launch dual laser merger
```bash
ros2 launch skratch_sensor dual_laser_merger.launch.py
```

---

## Configuration

Edit the parameter file:
```bash
nano ~/ros_robots/skratch_ws/src/skratch_sensor/config/dual_laser_merger_params.yaml
```

Typical parameters include:
- `laserscan_topics` - List of input laser scan topics
- `destination_frame` - Target TF frame for merged scan
- `angle_min`, `angle_max` - Angular range limits
- `range_min`, `range_max` - Distance range limits

**Ensure that:**
- TF transforms for both laser frames are available
- Both scanners publish at compatible frequencies