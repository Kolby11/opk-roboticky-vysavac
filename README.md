### Dependencies
 - opencv2


nix develop
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
- `/map` (`nav_msgs/msg/OccupancyGrid`) publishes the loaded PNG map.
- `/odom` (`nav_msgs/msg/Odometry`) publishes robot pose and velocity.
- `/tf` publishes `odom -> base_link`.
- `/robot_markers` (`visualization_msgs/msg/MarkerArray`) shows the robot body and heading.
- `/scan` (`sensor_msgs/msg/LaserScan`) publishes lidar readings.
- `/environment_markers` (`visualization_msgs/msg/MarkerArray`) shows obstacles and station.
