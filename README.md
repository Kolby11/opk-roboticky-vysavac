### Dependencies
 - opencv2


nix develop
cmake -S . -B build
cmake --build build

### RViz

Run the simulator, then open RViz with the included display config:

```sh
./build/program resources/config.yml
rviz2 -d resources/robot.rviz
```

RViz topics:

- `/cmd_vel` (`geometry_msgs/msg/Twist`) controls the robot.
- `/odom` (`nav_msgs/msg/Odometry`) publishes robot pose and velocity.
- `/tf` publishes `odom -> base_link`.
- `/scan` (`sensor_msgs/msg/LaserScan`) publishes lidar readings.
- `/environment_markers` (`visualization_msgs/msg/MarkerArray`) shows obstacles and station.
