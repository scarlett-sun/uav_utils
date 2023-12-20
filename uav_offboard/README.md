
### Usage
`waypoint_publisher_circular.cpp`
```txt
Usage: waypoint_publisher <namespace> <radius_d> <x_sen> <y_sen> <z_sen> <z_d> <pitch_sen> <pitch_d> <yaw_sen> <yaw_d> <loop_d> [<motion duration>]
```

```bash
rosrun uav_offboard waypoint_publisher_circular __ns:=/ardrone 2 0 0 1 1 0 0 0 360 1 10
```