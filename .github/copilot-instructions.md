# Global Planner Repository - AI Agent Instructions

## Project Overview

This is a ROS2 robotics project implementing **three core subsystems** for UGV (Unmanned Ground Vehicles):
1. **Global Path Planning** - A* algorithm with Bezier path smoothing on 3D occupancy grids
2. **Relocalization & Localization** - Two-stage coarse-to-fine ICP with IESKF Kalman filtering for LiDAR+IMU sensor fusion
3. **Service Interface** - ROS2 `.srv` definitions enabling cross-module communication

The codebase runs on embedded ARM64 systems (aarch64 cross-compilation hardcoded in `src/global_planner/CMakeLists.txt`) and uses ROS2 as the middleware layer. **Key design**: Non-ROS core libraries (`src/localizer/`, `src/global_planner/`) wrapped by ROS2 nodes for modularity.

## Architecture

### Core Components & Responsibilities
- **`src/localizer/`** - Relocalization & localization subsystem (non-ROS library + ROS2 wrapper)
  - **Core**: `localizer.h/cpp` - Non-ROS interface with YAML-driven config, callback-based pose updates
  - **Algorithms**: `localizers/{icp_localizer.cpp, ieskf.cpp, imu_processor.cpp, lidar_processor.cpp}`
  - **Data**: `ikd_Tree.cpp` (KD-tree for fast map search), `commons.cpp` (shared utilities like `esti_plane()`)
  - **ROS2 Bridge**: `localizer_node.cpp` - Subscribes to LiDAR + IMU topics, publishes TF2 transforms and debug clouds
- **`src/global_planner/`** - Path planning library (non-ROS)
  - **Algorithm**: `A_star.cpp` - 3D grid search, GPS→UTM conversion
  - **Map**: `occupy_map.cpp` - Voxel-based 3D occupancy grid with inflation
  - **Smoothing**: `Party/` - Bezier curve path smoothing
- **`src/interface/`** - ROS2 message definitions
  - `srv/Relocalize.srv` - Triggers relocalization with pose guess (x, y, z, yaw, pitch, roll)

### Data Flow (Localization Pipeline)
```
Raw LiDAR Cloud + Raw IMU Data
  ↓
LidarProcessor::undistort() [motion compensation via IMU]
  ↓
IESKF Predict Phase [velocity/gravity propagation]
  ↓
ICPLocalizer::align() [two-stage: rough → fine]
  ↓
IESKF Update Phase [residual correction]
  ↓
TF2 broadcast (map→lidar) + PoseStamped callback → visualization
```

## Key Build Patterns

### Build & Deployment
```bash
# From ROS2_WS root, build only localizer subsystem:
cd /home/zxhc/Workspace/ROS2_WS
colcon build --packages-select localizer

# Run with launch file (loads YAML config):
ros2 launch localizer localizer_launch.py
```

### Critical Dependencies
- **Sophus** (v1.22.10): Lie algebra library. MUST compile with `-DSOPHUS_USE_BASIC_LOGGING=ON` (set in `CMakeLists.txt:add_compile_definitions()`)
- **PCL** (with aarch64 cross-compilation paths hardcoded in `src/global_planner/CMakeLists.txt`)
- **Eigen3, YAML-cpp, ROS2 sensor_msgs, geometry_msgs, message_filters**

### ARM64 Cross-Compilation
`src/global_planner/CMakeLists.txt` contains hardcoded cross-compiler paths:
```cmake
set(CMAKE_C_COMPILER /usr/bin/aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER /usr/bin/aarch64-linux-gnu-g++)
set(Eigen3_INCLUDE_DIR "/home/zxhc/cross/install/aarch64/include/eigen3")
```
**Don't remove these** - they're intentional for embedded ARM targets.

## Configuration & Runtime

### YAML-Driven Parameter Pattern
All runtime config loaded from `YAML::LoadFile()` in `Localizer::Localizer()` and `LocalizerNode::loadParameters()`. Example structure (`src/localizer/config/localizer.yaml`):

```yaml
# Sensor topics & frames
cloud_topic: /livox/lidar          # LiDAR input subscription
odom_topic: /livox/imu             # IMU input subscription
map_frame: map
local_frame: lidar

# IESKF Kalman filter tuning
update_hz: 1.0                     # Filter update rate
na: 0.01                           # Accelerometer noise std
ng: 0.01                           # Gyroscope noise std
nba: 0.0001                        # Accel bias noise
nbg: 0.0001                        # Gyro bias noise
imu_init_num: 20                   # IMU samples to average during initialization

# LiDAR preprocessing
lidar_filter_num: 6                # Downsample period
lidar_min_range: 0.5               # Minimum valid range (meters)
lidar_max_range: 30.0              # Maximum valid range
scan_resolution: 0.15              # Voxel size for current scan
map_resolution: 0.3                # Voxel size for map

# ICP Stage 1: Coarse alignment (larger voxels, fewer iterations)
rough_scan_resolution: 0.25
rough_map_resolution: 0.25
rough_max_iteration: 5
rough_score_thresh: 0.2            # Fitnessiness score threshold (relaxed)

# ICP Stage 2: Fine alignment (finer resolution, strict scoring)
refine_scan_resolution: 0.1
refine_map_resolution: 0.1
refine_max_iteration: 10
refine_score_thresh: 0.1           # Stricter fitness threshold

# Lidar-IMU calibration (factory set)
r_il: [1.0, 0.0, 0.0, ...]        # 3x3 rotation matrix (row-major) Lidar→IMU
t_il: [-0.011, -0.02329, 0.04412] # Translation vector Lidar→IMU

# Global map path
pcd_path: /path/to/map.pcd         # Prebuilt point cloud for relocalization
```

**Pattern**: Always load config in constructor, store in `Config` struct, pass to component constructors (IMU/LiDAR processors, ICP localizer). No global singletons.

## Code Patterns & Conventions

### Non-ROS Library Interface Pattern
Core algorithm lives in **non-ROS classes** (e.g., `Localizer`, `GlobalPlanner`) accepting YAML config path in constructor:
```cpp
// src/localizer/src/localizer.h
class Localizer {
  explicit Localizer(const std::string &config_path);
  void feedIMU(const IMUData &imu);           // Producer thread pushes data
  void feedLidar(const PointCloud &cloud);
  void setPoseCallback(PoseCallback cb);      // Callback sink for results
  void setLogCallback(LogCallback cb);
};
```

**Why**: Decouples algorithm from ROS2 middleware, enables embedded/non-ROS deployments.

### ROS2 Node Wrapping Pattern
`LocalizerNode` wraps the library:
1. Load YAML config → construct `Localizer` 
2. Subscribe to ROS2 topics (LiDAR, IMU) → `feedIMU()`/`feedLidar()` calls
3. Receive pose via callback → publish TF2 + `/odometry` topic
4. Service handler (`/localizer/relocalize`) → `setPose()` to force relocalization

### Sensor Fusion Sequence
**IMU Initialization** (first 20 samples):
1. `IMUProcessor::initialize()` - Accumulate raw IMU, compute gravity direction
2. Enable localization only after threshold crossed

**Per-LiDAR-Frame Update**:
1. `LidarProcessor::undistort()` - Motion compensation using IMU gyro + previous velocity
2. Downsample current scan to `scan_resolution` voxels
3. `ICPLocalizer::align()` - Two-stage registration against map
   - **Stage 1** (rough): `rough_scan_resolution`, `rough_max_iteration=5` → coarse alignment
   - **Stage 2** (fine): `refine_scan_resolution`, `refine_max_iteration=10` → precise correction
4. `IESKF::predict()` - Propagate state forward via IMU acceleration
5. `IESKF::update()` - Correct pose using ICP residuals as measurement

### Callback Pattern (Non-ROS Event Handling)
```cpp
using PoseCallback = std::function<void(const PoseStamped &)>;
localizer->setPoseCallback([](const PoseStamped &pose) { 
  // Called every time pose updates (async-safe)
});
```

### Multi-Threading Safety
- `IMU/LiDAR` data stored in thread-safe `std::deque` with `std::mutex`
- Processors run in background threads (can be spawned via `startAsync()`)
- **No atomic operations** - locks protect full read-modify-write cycles

## File Organization

```
global_planner/              # Repository root
├── src/
│   ├── localizer/              # Main relocalization & localization subsystem
│   │   ├── CMakeLists.txt
│   │   ├── config/
│   │   │   └── localizer.yaml              # Runtime tuning parameters
│   │   ├── launch/
│   │   │   └── localizer_launch.py         # ROS2 launch script
│   │   ├── src/
│   │   │   ├── localizer.h/cpp             # Non-ROS library core
│   │   │   ├── localizer_node.cpp          # ROS2 wrapper node
│   │   │   ├── localizer_utils.h/cpp       # Utilities (math, transforms)
│   │   │   ├── localizers/
│   │   │   │   ├── commons.cpp             # Shared utilities (plane fitting, etc)
│   │   │   │   ├── icp_localizer.h/cpp     # Two-stage ICP registration engine
│   │   │   │   ├── ieskf.h/cpp             # Error-State Kalman Filter core
│   │   │   │   ├── ikd_Tree.h/cpp          # KD-tree for fast nearest neighbor
│   │   │   │   ├── imu_processor.h/cpp     # IMU buffer, undistortion, prediction
│   │   │   │   └── lidar_processor.h/cpp   # LiDAR buffer, downsampling
│   │   │   └── Party/                      # Third-party: Bezier path smoothing
│   │   └── rviz/
│   │       └── localizer.rviz              # Pre-configured visualization layout
│   │
│   ├── global_planner/         # Path planning subsystem
│   │   ├── CMakeLists.txt
│   │   ├── include/
│   │   │   ├── global_planner.h            # Public API
│   │   │   ├── A_star.h                    # A* algorithm interface
│   │   │   ├── occupy_map.h                # 3D occupancy grid
│   │   │   └── planer_utils.h              # GPS↔UTM conversion, utilities
│   │   └── src/
│   │       ├── global_planner.cpp          # Wrapper class, Bezier smoothing
│   │       ├── A_star.cpp                  # Core A* implementation
│   │       ├── occupy_map.cpp              # Voxel grid implementation
│   │       └── Party/                      # Bezier curve generation
│   │
│   └── interface/              # ROS2 message definitions (shared)
│       ├── CMakeLists.txt
│       ├── package.xml
│       └── srv/
│           └── Relocalize.srv              # Service: trigger relocalization
│
├── bag/                        # Recorded rosbag2 datasets for testing
│   └── rosbag2_2024_06_20-16_46_47/
│
├── build/                      # colcon build output
├── install/                    # colcon install output (executables, libraries)
├── log/                        # Build/runtime logs
│
├── CMakeLists.txt             # Top-level cmake (if mono-repo style)
├── README.md                  # Chinese documentation
├── .github/
│   └── copilot-instructions.md # This file
└── .gitignore
```

## Critical Developer Workflows

### Build & Test
```bash
# Build entire workspace (includes global_planner, localizer, interface)
cd /home/zxhc/Workspace/ROS2_WS
colcon build

# Build only localizer module (faster iterative development)
colcon build --packages-select localizer

# Source environment variables
source /home/zxhc/Workspace/ROS2_WS/install/setup.bash

# Run localizer node with default YAML config
ros2 launch localizer localizer_launch.py

# Test relocalization service (set initial pose guess)
ros2 service call /localizer/relocalize interface/srv/Relocalize \
  "{x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}"

# Inspect TF2 tree during operation
ros2 run tf2_tools view_frames

# Play recorded bag file for offline testing
ros2 bag play /path/to/rosbag2_2024_06_20-16_46_47
```

### Debugging Localization Failures
1. **Check ICP convergence**: Subscribe to `/localizer/map_cloud` and `/localizer/body_cloud` in RViz
   - If point clouds don't overlap → relocalization failed or initial pose too far off
2. **Verify IMU initialization**: Check logs for "IMU initialized" message
   - Delay localizer by manually calling service after robot settles
3. **Tune ICP scoring**:
   - Decrease `rough_score_thresh` (e.g., 0.15) if coarse stage skips valid alignments
   - Increase `refine_max_iteration` (e.g., 15) if fine stage terminates prematurely
4. **Check frame transforms**: `ros2 topic echo /tf | grep map` to verify `map→lidar` broadcasts

### Adding New Sensor Types
1. Define `.msg` file in `src/interface/msg/`
2. Add ROS2 dependency in `CMakeLists.txt` and `package.xml`
3. Extend `Localizer` feeders: `void feedSensor(const SensorMsg &msg)`
4. Integrate in `LocalizerNode::onSensorCallback()`

### Modifying Path Planning Algorithm
1. Core algorithm in `src/global_planner/src/A_star.cpp` - implements `search()` on 3D occupancy grid
2. GPS↔UTM conversion in `src/global_planner/src/global_planner.cpp` (uses `GeographicLib` third-party)
3. Path smoothing in `src/global_planner/src/Party/` (Bezier curves)
4. To swap algorithm: Implement `IPlanningAlgorithm` interface, instantiate in `GlobalPlanner::init()`

## Build System & Key Dependencies

### CMake Configuration (`src/localizer/CMakeLists.txt`)
```cmake
# C++17 standard (structured bindings, optional used throughout)
set(CMAKE_CXX_STANDARD 17)

# Critical: Sophus depends on fmt library unless this is set
add_compile_definitions(SOPHUS_USE_BASIC_LOGGING)

# Multi-threading support (message_filters, async processing)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

# Aggressive optimization for embedded deployment
add_compile_options(-O3)
```

### Dependencies & Their Roles
| Dependency | Purpose | Version/Notes |
|---|---|---|
| **Sophus** | Lie algebra (SO(3), SE(3) transforms) | v1.22.10, MUST compile with `-DSOPHUS_USE_BASIC_LOGGING=ON` |
| **PCL** | Point cloud processing, KD-tree | Standard ROS2 release |
| **Eigen3** | Linear algebra, matrix operations | Implicit via PCL |
| **yaml-cpp** | Configuration file parsing | All parameters loaded via YAML |
| **ROS2 core** | Middleware (message_filters, tf2, rclcpp) | Standard |
| **message_filters** | Time-synchronized LiDAR+IMU subscription | Approximate time policy used |

### ARM64 Cross-Compilation (IMPORTANT)
`src/global_planner/CMakeLists.txt` contains **hardcoded cross-compiler paths**:
```cmake
set(CMAKE_C_COMPILER /usr/bin/aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER /usr/bin/aarch64-linux-gnu-g++)
set(Eigen3_INCLUDE_DIR "/home/zxhc/cross/install/aarch64/include/eigen3")
set(PCL_INCLUDE_DIRS "/home/zxhc/cross/install/aarch64/include")
```
**⚠️ Do NOT remove** - intentional for embedded ARM target deployment. These may fail on non-cross-compile systems (ignore CMake errors on host x86_64 machine).

## Testing & Validation

**Current Status**: Manual testing only (no unit test suite in codebase—see future roadmap in README).

### Offline Validation with Recorded Data
```bash
# Play recorded bag file at normal speed
ros2 bag play /path/to/bag/ --clock

# In another terminal, launch localizer
ros2 launch localizer localizer_launch.py

# Monitor localization in RViz
rviz2 /home/zxhc/Workspace/ROS2_WS/global_planner/src/localizer/rviz/localizer.rviz
```

### Manual Validation Checklist
- [ ] **TF2 Transforms**: `ros2 run tf2_tools view_frames.py` shows `map→lidar` chain during operation
- [ ] **Topic Publishing**: `ros2 topic list` includes `/localizer/body_cloud`, `/localizer/map_cloud`, `/odom`
- [ ] **Service Availability**: `ros2 service list | grep relocalize` shows `/localizer/relocalize`
- [ ] **RViz Visualization**: Point clouds overlap (visual alignment check)
- [ ] **Log Output**: `ros2 launch ... --log-level=debug` shows IMU initialization messages

### Debug Topics
- `/localizer/body_cloud` - Current LiDAR scan in lidar frame
- `/localizer/map_cloud` - Map points near current pose
- `/localizer/refined_cloud` - Point cloud after fine ICP stage
- `/odom` - PoseStamped odometry broadcast

## Integration Points & API Surface

### ROS2 Service API
```
Service: /localizer/relocalize
Type: interface/srv/Relocalize
Params: x, y, z (meters), yaw, pitch, roll (radians)
Returns: success (bool), message (string)
```

### Library API (Non-ROS Usage)
```cpp
#include "localizer/localizer.h"
auto localizer = std::make_shared<localizer::Localizer>("config.yaml");
localizer->setPointCloud("/path/to/map.pcd");
localizer->setPoseCallback([](const auto &pose) { /*...*/ });
localizer->feedIMU(imu_data);
localizer->feedLidar(point_cloud);
```

### Global Planner API
```cpp
#include "global_planner/global_planner.h"
auto planner = std::make_shared<GlobalPlanner>("config.yaml");
Path smoothed_path = planner->plan(start, goal); // Returns Bezier-smoothed path
```

---

**Last Updated:** 2025-11-26  
**Branch:** `non_ros_version`  
**Status:** Production-Ready (core localization) | Future: SLAM integration, multi-algorithm support  
**Maintainer Notes:** Keep ARM64 cross-compile flags in `src/global_planner/CMakeLists.txt` for embedded deployment
