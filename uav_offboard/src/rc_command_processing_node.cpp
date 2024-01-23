#include "uav_offboard/rc_command_processing_node.h"

RcCommandProcessingNode::RcCommandProcessingNode(const ros::NodeHandle& nh):nh_(nh),private_nh_("~") {
  trajectory_point_msg_.points.resize(1);
  trajectory_point_msg_.points[0].transforms.resize(1);
  trajectory_point_msg_.points[0].velocities.resize(1);
  trajectory_point_msg_.points[0].accelerations.resize(1);

  InitializeParams();

  timer_ = nh_.createTimer(ros::Duration(dt_), &RcCommandProcessingNode::TimedCommandCallback, this,false, false);//timer
  
  trajectory_point_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command/trajectory",10);//trajectory point publisher
  // setpoint_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",10);
  setpoint_raw_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",10);
  attitude_euler_pub_ = nh_.advertise<geometry_msgs::Vector3>("attitude_euler",10);
  
  get_rc_channel_sub_ = nh_.subscribe("/mavros/rc/in",100, &RcCommandProcessingNode::RcInCallback,this);// rc signal subscriber
  local_pos_sub_ = nh_.subscribe("mavros/local_position/odom", 10, &RcCommandProcessingNode::OdometryCallback,this);// odom subscriber
  state_sub_ = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, &RcCommandProcessingNode::StateCallback,this);

  arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");  
}

RcCommandProcessingNode::~RcCommandProcessingNode(){};

/*Initialize some data members*/
void RcCommandProcessingNode::InitializeParams(){
  sw_arm_ = 1000;
  sw_kill_ = 1000;
  sw_position_attitude_ = 1000;
  joystick_value_ = Eigen::Vector4i::Zero();
  thrust_ = joystick_value_[2];
  thrust_last_time_ = thrust_;

  euler_vel_ = Eigen::Vector3d::Zero();

  is_armed_ = false;
  is_killed_ = false;
  is_normal_ = true;
  position_or_attitude_ = true;//default position mode
  need_offboard_ = false;
  
  GetChannelConfiguration(private_nh_,ch_config_);
  GetMaxVelocity(private_nh_,twist_);
  dt_ = kDefaultDt;	
  GetDeltaT(private_nh_,dt_);
  GetMode(private_nh_,need_offboard_);

  count_ = 0;
  thrust_mid_ = ch_config_.channels.at(2).mid;

  offb_set_mode_srv_.request.custom_mode = "OFFBOARD";
  arm_cmd_srv_.request.value = true;
}


void RcCommandProcessingNode::StateCallback(const mavros_msgs::State::ConstPtr& msg){
    current_state_ = *msg;
}

/*Use the initial pose as the first trajectory setpoint command*/
void RcCommandProcessingNode::OdometryCallback(const nav_msgs::OdometryConstPtr& msg){
  if(count_==0){//get the initial pose as the first trajectory point command
	pos_last_.x = msg->pose.pose.position.x;
	pos_last_.y = msg->pose.pose.position.y;
	pos_last_.z = msg->pose.pose.position.z;
  pos_setpoint_ = pos_last_;
	GetEulerAnglesFromQuaternion(msg->pose.pose.orientation,euler_last_);
  att_setpoint_ = euler_last_;
  }
  if(!is_armed_){
    // SetAndPubGeometryPose();//try, do not use setpoint_position
    SetAndPubPositionRaw();
  }//do not publish after arm
  // As long as the joystick does push above middle point
  // the local_position received from mavros will in turn serve as setpoint_position command
}

/*Process remote controller signals, need to be customized*/
void RcCommandProcessingNode::RcInCallback(const mavros_msgs::RCInConstPtr& msg){
  sw_arm_ = msg->channels[13];//the real number is 6
  sw_kill_ = msg->channels[5];
  sw_position_attitude_ = msg->channels[4];
  thrust_last_time_ = thrust_;
  thrust_ = msg->channels[2];
  
  for(int i =0 ; i<4; i++){
	joystick_value_[i]=msg->channels[i];
  }

  // comment the below four lines since for now we only consider position mode
  // if(msg->channels[8]<1500){
	// position_or_attitude_ = true;//position command
  // }else{
	// position_or_attitude_ = false;//attitude command
  // }

  SetArmKillInfo();
}

/*Publish trajectory setpoint command at the given rate*/
void RcCommandProcessingNode::TimedCommandCallback(const ros::TimerEvent& e){
  MapJoystickToVel();
  ComputeSetpoint();
  SetAndPubTrajectoryPoint();
  // SetAndPubGeometryPose();try, do not set setpoint_position msg
  SetAndPubPositionRaw();//set setpoint_raw msg
  attitude_euler_pub_.publish(att_setpoint_);//for debug only
}

/*Set current arm & kill combination state, need to be customized*/
void RcCommandProcessingNode::SetArmKillInfo(){
  if(sw_kill_ < 1500 && sw_arm_ > 1500){//do no kill, do arm
    // ROS_INFO_STREAM("arm state");
    is_killed_ = false;
    // is_armed_ = true;
    is_normal_ = false;

    if(need_offboard_){
      if( current_state_.mode != "OFFBOARD"){
        if( set_mode_client_.call(offb_set_mode_srv_) &&offb_set_mode_srv_.response.mode_sent){
          ROS_INFO("Offboard enabled");
        }
      } 
      else {
        if(!current_state_.armed){
          if( arming_client_.call(arm_cmd_srv_) && arm_cmd_srv_.response.success){
            is_armed_ = true;
            ROS_INFO("Vehicle armed");
          }
        }
      }
    } 
    else {
      is_armed_ = true;
      ROS_INFO("Vehicle armed");
    }
  }
  else if((sw_kill_ > 1500 && sw_arm_ > 1500)
  			||(sw_kill_ > 1500 && sw_arm_ < 1500)){//do kill, whether do arm or not
    // ROS_INFO_STREAM("kill state");
    is_armed_ = false;
    is_killed_ = true;
    is_normal_ = false;
  }
  else if(sw_kill_ < 1500 && sw_arm_ < 1500){//do no kill,do no arm
    // ROS_INFO_STREAM("normal state");
    is_armed_ = false;
    is_killed_ = false;
    is_normal_ = true;
  }

  //judge if the thrust is up to middle
  if(is_armed_==true && thrust_last_time_ <= thrust_mid_ && thrust_ >= thrust_mid_){
	// ROS_INFO_STREAM("armed and the uav will fly..");
	count_ ++;
	if(count_>2){count_=2;}//prevent overflow

	if(count_ == 1){//will execute only once
	//   ROS_INFO_STREAM("init controller ....");
	  pos_setpoint_ = pos_last_;
    att_setpoint_ = euler_last_;

    pos_velocity_setpoint_.x = 0;	pos_velocity_setpoint_.y = 0;	pos_velocity_setpoint_.z = 0;
	  att_velocity_setpoint_.x = 0;	att_velocity_setpoint_.y = 0;	att_velocity_setpoint_.z = 0;
    pos_acc_setpoint_.x = 0;      pos_acc_setpoint_.y = 0;      pos_acc_setpoint_.z = 0;
	  SetAndPubTrajectoryPoint();
    // SetAndPubGeometryPose();//try, do not set setpoint_position msg
    SetAndPubPositionRaw();
	  timer_.start();//start the timer
	}
  }
  else if (is_killed_==true || is_normal_==true){
	// ROS_INFO_STREAM("killed or normal");
	count_ = 0;
	timer_.stop();
  }	
}

/*Map joystick values to velocities, need to be customized*/
void RcCommandProcessingNode::MapJoystickToVel(){
  double vel_factor[]={0.0, 0.0, 0.0, 0.0};
  for(int i = 0; i < 4;i++){
	if((float)(abs(joystick_value_[i]-ch_config_.channels[i].mid))>15){//joystick 2 的阈值是15还是25，注意一下，会影响到摇杆的灵敏度。
	  vel_factor[i]= - ((float)(joystick_value_[i]-ch_config_.channels[i].mid)/
					(float)(ch_config_.channels[i].max - ch_config_.channels[i].mid));//channel i
	}
  }
  vel_factor[2] = - vel_factor[2];// vz_factor, FLU Frame

  if(position_or_attitude_){//position mode
	twist_.v[0] = vel_factor[1] * twist_.v_max[0];//vx
	twist_.v[1] = vel_factor[3] * twist_.v_max[1];//vy
	twist_.v[2] = vel_factor[2] * twist_.v_max[2];//vz
	twist_.omega[2] = vel_factor[0] * twist_.omega_max[2];//omega_z
  } else{//attitude mode
	vel_factor[3] = - vel_factor[3];// omega_x_factor, FLU Frame
	twist_.omega[0] = vel_factor[3] * twist_.omega_max[0];//omega_x
	twist_.omega[1] = vel_factor[1] * twist_.omega_max[1];//omega_y
	twist_.omega[2] = vel_factor[0] * twist_.omega_max[2];//omega_z
	twist_.v[2] = vel_factor[2] * twist_.v_max[2];//vz
  }
  //transform angular velocity to euler angular rate
  euler_vel_[0] = twist_.omega[0] + 
				  sin(euler_last_.x) * tan(euler_last_.y) * twist_.omega[1] + 
				  cos(euler_last_.x) * tan(euler_last_.y) * twist_.omega[2];
  euler_vel_[1] = cos(euler_last_.x) * twist_.omega[1] -
				  sin(euler_last_.x) * twist_.omega[2];
  euler_vel_[2] = sin(euler_last_.x)/cos(euler_last_.y) * twist_.omega[1] + 
				  cos(euler_last_.x)/cos(euler_last_.y) * twist_.omega[2];
}

/*Compute the trajectory point command to be published*/
void RcCommandProcessingNode::ComputeSetpoint(){
  pos_setpoint_.x = pos_last_.x + twist_.v[0]*dt_;
  pos_setpoint_.y = pos_last_.y + twist_.v[1]*dt_;
  pos_setpoint_.z = pos_last_.z + twist_.v[2]*dt_;

  att_setpoint_.x = euler_last_.x + euler_vel_[0]*dt_;
  att_setpoint_.y = euler_last_.y + euler_vel_[1]*dt_;
  att_setpoint_.z = euler_last_.z + euler_vel_[2]*dt_;

  pos_last_ = pos_setpoint_;
  euler_last_ = att_setpoint_;

  pos_velocity_setpoint_.x = twist_.v[0];
  pos_velocity_setpoint_.y = twist_.v[1];
  pos_velocity_setpoint_.z = twist_.v[2];
  att_velocity_setpoint_.x = twist_.omega[0];
  att_velocity_setpoint_.y = twist_.omega[1];
  att_velocity_setpoint_.z = twist_.omega[2];
}

/*Set and publish trajectory point message*/
void RcCommandProcessingNode::SetAndPubTrajectoryPoint(){
  trajectory_point_msg_.points[0].transforms[0].translation = pos_setpoint_;
  trajectory_point_msg_.points[0].transforms[0].rotation = tf::createQuaternionMsgFromRollPitchYaw
  															(att_setpoint_.x, att_setpoint_.y, att_setpoint_.z);
  trajectory_point_msg_.points[0].velocities[0].linear = pos_velocity_setpoint_;
  trajectory_point_msg_.points[0].velocities[0].angular = att_velocity_setpoint_;
  
  trajectory_point_pub_.publish(trajectory_point_msg_);
}


// /*Set and publish geometry_msg::posestamped message*/
// void RcCommandProcessingNode::SetAndPubGeometryPose(){
//   pose_stamped_msg_.pose.position.x = pos_setpoint_.x;
//   pose_stamped_msg_.pose.position.y = pos_setpoint_.y;
//   pose_stamped_msg_.pose.position.z = pos_setpoint_.z;
//   pose_stamped_msg_.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw
//   															(att_setpoint_.x, att_setpoint_.y, att_setpoint_.z);
//   pose_stamped_msg_.header.stamp = ros::Time::now();
  
//   setpoint_pos_pub_.publish(pose_stamped_msg_);
// }


/*Set and publish geometry_msg::posestamped message*/
void RcCommandProcessingNode::SetAndPubPositionRaw(){
  position_target_msg_.header.stamp = ros::Time::now();

  position_target_msg_.position.x = pos_setpoint_.x;
  position_target_msg_.position.y = pos_setpoint_.y;
  position_target_msg_.position.z = pos_setpoint_.z;
  position_target_msg_.velocity = pos_velocity_setpoint_;
  position_target_msg_.acceleration_or_force = pos_acc_setpoint_;

  position_target_msg_.yaw = att_setpoint_.z;
  position_target_msg_.yaw_rate = att_velocity_setpoint_.z;
  position_target_msg_.coordinate_frame = 1;//LOCAL_NED

  setpoint_raw_pub_.publish(position_target_msg_);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "rc_command_processing_node");
  ros::NodeHandle nh;
  RcCommandProcessingNode RcCommandProcessor(nh);
  ros::spin();
  return 0;
}