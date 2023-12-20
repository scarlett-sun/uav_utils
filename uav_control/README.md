### 需求
* 将命名空间从rotors_control修改为uav_control，以示与原代码区分。

* 将ComputeDesiredAcceleration函数增加了惯性矩阵，应该算是一种修正，原来的代码有问题。

* 原来是用[角加速度；拉力]，通过一个分配矩阵转换为电机的角速度。然后修改为了，用[力矩；拉力]通过分配矩阵转换为电机角速度。

* 在指令接收方面，有两种方式，一种是一次一次地发送指令；另一种是一次性发送所有规划好的轨迹点，然后按照轨迹点的实际间隔时间设置定时器的频率。

* 将回调触发改为定时器触发。接收到第一条指令后，触发定时器，会定时执行OverallController()。

* 新增力和力矩的发布函数。已写，发给mavros，但是话题还没确定。

* 新增部署到硬件的yaml参数文件。
    * 控制器时间间隔 0.01    
    * string 常用话题
    * float 拉力缩放系数（注释里写说明）50
    * bool 使用定时器回调或者消息回调（暂时不考虑这项功能）
    * bool auto_arm（暂时不考虑这项功能）

* 检查消息频率是否都为100Hz
* 为每一次能正常function的提交写使用文档。

### Usage
1. Launch a hovering example
```bash
roslaunch uav_offboard sim_mav_hovering_example.launch
``` 

2. Circular trajectory tracking(see package uav_offboard README.md for detail)
```bash
rosrun uav_offboard waypoint_publisher_circular __ns:=/ardrone 2 0 0 1 1 0 0 0 360 1 10
```


