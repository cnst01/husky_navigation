# husky_navigation

ROS2 package for Husky navigation (Nav2 demos, waypoint state machine).

## Build (inside workspace root)
```bash
cd ~/clearpath_ws
colcon build --symlink-install
source install/setup.bash
```

## Run
- Launch navigation stack:
```bash
ros2 launch husky_navigation nav2_husky.launch.py
```
- Run waypoint state machine:
```bash
ros2 run husky_navigation waypoint_state_machine.py
```