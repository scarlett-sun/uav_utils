
### Usage
#### Trajectory Generation
`waypoint_publisher_circular.cpp`
```txt
Usage: waypoint_publisher <namespace> <radius_d> <x_sen> <y_sen> <z_sen> <z_d> <pitch_sen> <pitch_d> <yaw_sen> <yaw_d> <loop_d> [<motion duration>]
```

```bash
rosrun uav_offboard waypoint_publisher_circular __ns:=/ardrone 2 0 0 1 1 0 0 0 360 1 10
```

#### Remote Controller Signal Interpretation and Processing
```cpp
// 数据成员初始化写的不是很好
// 刚arm之后会有z position 会有锯齿状波形，不合理，逻辑问题已改，ok。
// position模式，yaw摇杆变换，没有反应，逻辑问题，ok
// yaml参数需要改，每个遥控器不一样。
// attitude 的x方向好像反了，ok
// 转成roll pitch yaw进行输出，现在这个四元数太不直观了，ok
// below midpoint才可以成功arm
// not kill, arm, let_fly=true/false, has above midpoint once --> let_fly=true
// kill,arm,let_fly=true/false --> let_fly = false, count_ = 0
// not kill, not arm, let_fly=true/false --> let_fly = false, count_ = 0
// kill, not arm,let_fly=true/false --> let_fly = false, count_ = 0
// 油门放在最低点不由程序判断，而是由遥控器自身报警或者用户自行判断。
// 遥控油门正反需要在遥控器内部设置，会影响到部分代码逻辑，需要在文档中说明。

// 纯代码来看，还差一个数据成员初始化和代码风格统一（google）,
// 校对max, min, mid，ok
// 接入单机进行gazebo, ok
// 考虑是否在yaml中增加zero_zone(15/25)
// 通信频率还没有检查。
// 在launch文件里写好接口示例
// clear or commented the ros info stream debug code
// 写好文档，对可能需要用户修改的代码逻辑进行说明
// Add foxglove panel布置说明
```

#### Offboard real flight under rc signal command
```bash
roslaunch mavros px4.launch
roslaunch uav_offboard rc_command_processing.launch
```

新增SE开关为fake arm（only used in offboard mode），增加ros service以实现软件arm和软件切off-board模式