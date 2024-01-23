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
#include "uav_offboard/rc_common.h"

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>


class RcCommandProcessingNode{
 public:
  RcCommandProcessingNode(const ros::NodeHandle& nh);
  ~RcCommandProcessingNode();

 private://node and callbacks
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  void TimedCommandCallback(const ros::TimerEvent& e);
  void OdometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void RcInCallback(const mavros_msgs::RCInConstPtr& msg);
  void StateCallback(const mavros_msgs::State::ConstPtr& msg);

 private://member functions
  void SetArmKillInfo();
  void MapJoystickToVel();
  void ComputeSetpoint();
  void SetAndPubTrajectoryPoint();
  // void SetAndPubGeometryPose();
  void SetAndPubPositionRaw();
  void InitializeParams();

 public://publishers and subscribers and timers
  ros::Publisher trajectory_point_pub_;//for rotors gazebo simulation
  // ros::Publisher setpoint_pos_pub_;//for offboard real flight, commented out, effect not good: large response delay 
  ros::Publisher setpoint_raw_pub_;//try this one
  ros::Publisher attitude_euler_pub_;//for debug only
  ros::Subscriber get_rc_channel_sub_;
  ros::Subscriber local_pos_sub_;
  ros::Subscriber state_sub_;

  ros::Timer timer_;

  ros::ServiceClient arming_client_;
  ros::ServiceClient set_mode_client_;

 private://msgs
  geometry_msgs::Vector3 pos_last_;//define setpoint last variables for integral last setpoint
  geometry_msgs::Vector3 euler_last_;//define setpoint last variables for integral last setpoint	
  geometry_msgs::Vector3 pos_setpoint_;//position setpoint
  geometry_msgs::Vector3 att_setpoint_;//attitude setpoint(euler)
  geometry_msgs::Vector3 pos_velocity_setpoint_;//linear velocity setpoint
  geometry_msgs::Vector3 pos_acc_setpoint_;
  geometry_msgs::Vector3 att_velocity_setpoint_;//angular velocity setpoint
  trajectory_msgs::MultiDOFJointTrajectory trajectory_point_msg_;//trajectory to send to the controller
  geometry_msgs::PoseStamped pose_stamped_msg_;
  mavros_msgs::PositionTarget position_target_msg_;
  mavros_msgs::State current_state_;

  mavros_msgs::SetMode offb_set_mode_srv_;
  mavros_msgs::CommandBool arm_cmd_srv_;

 private:
  /*---------RCIn--------*/
  int sw_arm_;//arm switch, LS
  int sw_kill_;//kill switch, RS
  int sw_position_attitude_;//SE, position command(x y z yaw) or attitude command(roll pitch yaw z) switch
  Eigen::Vector4i joystick_value_=Eigen::Vector4i::Zero();//J1-J4
  /*---------RCIn--------*/

  Eigen::Vector3d euler_vel_ = Eigen::Vector3d::Zero();

  bool is_armed_;
  bool is_killed_;
  bool is_normal_;
  bool position_or_attitude_;
  
  Twist twist_;
  ChannelConfiguration ch_config_;

  int thrust_;//Joystick J2
  int thrust_last_time_=0;
  int count_ = 0;// the first time J2 go above middle will count_ become 1

  float dt_;
  int thrust_mid_;
  bool need_offboard_;
  };
#endif
