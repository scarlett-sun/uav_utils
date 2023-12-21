#ifndef INCLUDE_UAV_OFFBOARD_RC_COMMAND_PROCESSING_NODE_H_
#define INCLUDE_UAV_OFFBOARD_RC_COMMAND_PROCESSING_NODE_H_


#include <fstream>
#include <iostream>

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <tf/tf.h>

#include <std_msgs/Bool.h>
#include <mavros_msgs/RCIn.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <nav_msgs/Odometry.h>
#include "uav_offboard/default_values.h"

class RcCommandProcessingNode{
 public:
  RcCommandProcessingNode(const ros::NodeHandle& nh);//构造函数，初始化订阅者和发布者，初始化一些参数
  ~RcCommandProcessingNode();

 private://node and callbacks
  ros::NodeHandle nh_;
  void TimedCommandCallback(const ros::TimerEvent& e);//定时发布轨迹点
  void LocalPositionCallback(const nav_msgs::OdometryConstPtr& msg);//订阅mavros发回来的传感器数据
  void RcInCallback(const mavros_msgs::RCInConstPtr& msg);//订阅mavros发回来的遥控器信号

 private://member functions
  void SetArmKillInfo();
  void MapJoystickToVel();
  void ComputeSetpoint();
  void SetAndPubTrajectoryPoint();
  void GetEulerAnglesFromQuaternion(const geometry_msgs::Quaternion& q, geometry_msgs::Vector3& euler_angles);

 public://publishers and subscribers and timers
  // publisher
  ros::Publisher trajectory_point_pub_;
  // subsribers
  ros::Subscriber get_rc_channel_sub_;
  ros::Subscriber local_pos_sub_;
  // timer
  ros::Timer timer_;
  //-------------------完成，写完setpoint那里，然后把nominal pub修改完，替换为trajectory_point_pub_。-------
  //---------kill的消息优先级最高，会直接被px4处理，无需经过off board逻辑---------
  //待办：中心arm之后，为什么，其他子机都会arm？
  //待办：写图文并茂的说明文档（和遥控器的对应关系，使用方法），在代码里写好函数和变量的注释
  //待办：写一个用于实际飞行的5dof_real_flight.launch文件（清理掉gazebo的内容，整理消息接口），里面把子机的话题修改为相应的mavros接口，或者把mavros的子机节点的话题，接入5dof_real_flight.launch
  //待办：所有参数yaml化，留一个仿真版本（用来控制gazebo环境的飞行器的），还有一个实飞版本（显然参数待调优）
  //子机测试，需要上电 --> mavlink使能控制器（额外的一对数传）--> 确保控制器开启-->单飞
  //断电，接入平台（不能上电接入平台）
  //测试需要一整天

 private://msgs
  geometry_msgs::Vector3 pos_last_;//define setpoint last variables for integral last setpoint
  geometry_msgs::Vector3 euler_last_;//define setpoint last variables for integral last setpoint	
  
  nav_msgs::Odometry initial_pose_; // the initial pose of the plane at the start of the controller
  geometry_msgs::Vector3 pos_setpoint_;//position setpoint
  geometry_msgs::Vector3 att_setpoint_;//attitude setpoint(euler)
  geometry_msgs::Vector3 pos_velocity_setpoint_;//linear velocity setpoint
  geometry_msgs::Vector3 att_velocity_setpoint_;//angular velocity setpoint
  trajectory_msgs::MultiDOFJointTrajectory trajectory_point_msg_;//trajectory to send to the controller

 private:
  /*---------RCIn--------*/
  int sw_arm_;//arm switch, LS
  int sw_kill_;//kill switch, RS
  int sw_position_attitude_;//position command(x y z yaw) or attitude command(roll pitch yaw z) switch
  int thrust_;//Joystick J2
  Eigen::Vector4i joystick_value_=Eigen::Vector4i::Zero();//
  /*---------RCIn--------*/

  Twist twist_;
  Eigen::Vector3d euler_vel_ = Eigen::Vector3d::Zero();

  std_msgs::Bool arm_bool_;//define arm_bool params
  std_msgs::Bool kill_bool_;
  std_msgs::Bool normal_bool_;
  bool position_or_attitude_;

  ChannelConfiguration ch_config_;
  //define sw class
  int thrust_last_time_=0;
  int count_ = 0;// only when the controller starts, //this value will become 1
  bool let_fly_ = false;//bool fly switch

  float dt_;
  int thrust_mid_;
  // bool init_controller_ = false; //bool init controller 这个是用来初始化轨迹生成节点的指令初值的，在我这里（孙嘉丽）不是初始化控制器
};
#endif

// 数据成员初始化写的不是很好