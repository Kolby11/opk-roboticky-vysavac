### Dependencies
 - opencv2


nix develop
source /opt/ros/kilted/setup.bash
cmake -S . -B build
cmake --build build

### RViz

```sh
source /opt/ros/kilted/setup.bash
./build/program resources/config.yml
rviz2 -d resources/robot.rviz
```

RViz topics:

- `/cmd_vel` (`geometry_msgs/msg/Twist`) controls the robot.
- `/robot_command` (`robot_assignment/msg/RobotCommand`) controls the robot with the project command message.
- `/robot_state` (`robot_assignment/msg/RobotState`) publishes structured robot state.
- `/map` (`nav_msgs/msg/OccupancyGrid`) publishes the loaded PNG map.
- `/odom` (`nav_msgs/msg/Odometry`) publishes robot pose and velocity.
- `/tf` publishes `odom -> base_link`.
- `/robot_markers` (`visualization_msgs/msg/MarkerArray`) shows the robot body and heading.
- `/waste_markers` (`visualization_msgs/msg/MarkerArray`) shows active waste.
- `/scan` (`sensor_msgs/msg/LaserScan`) publishes lidar readings.
- `/game_state` (`robot_assignment/msg/GameState`) publishes structured Keep Clean state.
- `/game_path` (`nav_msgs/msg/Path`) publishes the travelled path.
- `/environment_markers` (`visualization_msgs/msg/MarkerArray`) shows obstacles and station.

ROS services:

- `/start_game` (`robot_assignment/srv/StartGame`) starts a new Keep Clean run.

### Web UI

The simulator exposes a browser API on port 8080 by default:

```sh
./build/program resources/config.yml
```

Run the Svelte app from another terminal:

```sh
cd web
npm install
npm run dev
```

Open the Vite URL, usually `http://localhost:5173`. Use WASD, arrow keys, or the on-screen controls.

Web API:

- `GET /api/scene` returns map metadata, obstacles, and station.
- `GET /api/state` returns robot pose, collision state, and lidar hits.
- `POST /api/command` accepts `{"linear": 60, "angular": 0}` velocity commands.
- `POST /api/game/restart` starts a fresh Keep Clean game.
- `GET /api/map-image` serves the configured PNG map.
