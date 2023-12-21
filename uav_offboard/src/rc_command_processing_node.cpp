#include "uav_offboard/rc_command_processing_node.h"

RcCommandProcessingNode::RcCommandProcessingNode(const ros::NodeHandle& nh){
  nh_=nh;
  arm_bool_.data =false;//init arm_bool to false to ensure safety
  kill_bool_.data = false;
  normal_bool_.data = false;
  position_or_attitude_ = false;

  trajectory_point_msg_.points.resize(1);
  trajectory_point_msg_.points[0].transforms.resize(1);
  trajectory_point_msg_.points[0].velocities.resize(1);
  trajectory_point_msg_.points[0].accelerations.resize(1);

  dt_ = kDefaultDt;

  GetChannelConfiguration(nh_,ch_config_);
  GetMaxVelocity(nh_,twist_);
  GetDeltaT(nh_,dt_);

  thrust_mid_ = ch_config_.channels.at(1).mid;//1还是2我忘了。
  timer_ = nh_.createTimer(ros::Duration(dt_), &RcCommandProcessingNode::TimedCommandCallback, this,false, false);//timer
  trajectory_point_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command/trajectory",10);//trajectory point publisher
  get_rc_channel_sub_ = nh_.subscribe("/mavros/rc/in",100, &RcCommandProcessingNode::RcInCallback,this);//订阅遥控器的mavros消息
  local_pos_sub_ = nh_.subscribe("mavros/local_position/odom", 10, &RcCommandProcessingNode::LocalPositionCallback,this);//订阅传感器状态odom
}

RcCommandProcessingNode::~RcCommandProcessingNode(){};
/*Set the initial pose of the plane*/
void RcCommandProcessingNode::LocalPositionCallback(const nav_msgs::OdometryConstPtr& msg){
	if(count_==0){
		initial_pose_ = *msg;
	}
}

/*遥控器mavros/rc/in信号*/
void RcCommandProcessingNode::RcInCallback(const mavros_msgs::RCInConstPtr& msg)
{
	sw_arm_ = msg->channels[6];// LD
	sw_kill_ = msg->channels[5]; // RD，原来是7，不知道为啥，因为默认是5
	sw_position_attitude_ = msg->channels[4];//SE
	thrust_last_time_ = thrust_;
	thrust_ = msg->channels[2];
	for(int i =0 ; i<4; i++){
		joystick_value_[i]=msg->channels[i];
	}
	SetArmKillInfo();
}

/*定时器回调*/
void RcCommandProcessingNode::TimedCommandCallback(const ros::TimerEvent& e){
	// std::cout << "started" << std::endl;
	ROS_INFO_ONCE("started");
	if(let_fly_){
		MapJoystickToVel();
		ComputeSetpoint();
		SetAndPubTrajectoryPoint();
	}
}

/*设置当前的arm kill的组合状态*/
void RcCommandProcessingNode::SetArmKillInfo(){
	if(sw_kill_ < 1500 && sw_arm_ > 1500)//no kill, arm
	{
		ROS_INFO_STREAM("arm");
		arm_bool_.data=true;
		kill_bool_.data = false;
		normal_bool_.data = false;
	}
	else if((sw_kill_ > 1500 && sw_arm_ > 1500)||(sw_kill_ > 1500 && sw_arm_ < 1500))//kill, whether arm or not
	{
		ROS_INFO_STREAM("kill");
		arm_bool_.data=false;
		kill_bool_.data = true;
		normal_bool_.data = false;
	}
	else if(sw_kill_ < 1500 && sw_arm_ < 1500)//no kill, no arm
	{
		ROS_INFO_STREAM("normal");
		arm_bool_.data=false;
		kill_bool_.data = false;
		normal_bool_.data = true;
	}

	//judge if the thrust is up to middle
	if(arm_bool_.data==true && thrust_last_time_ >= thrust_mid_ && thrust_ <= thrust_mid_)
	{
		ROS_INFO_STREAM("armed and the uav will fly..");
		let_fly_ = true;
		// init_controller_ = true;// will pub point
		count_ ++;
		std::cout <<"count: "<< count_<<std::endl;
	}
	else if (arm_bool_.data==false /* condition */)
	{
		ROS_INFO_STREAM("not arm..");
		let_fly_ = false;
		// init_controller_ = false;
		count_ = 0;
	}	

	// if(init_controller_ && count_ == 1){//只执行一次
	if(count_ == 1){//只执行一次
		// init_controller_ =false;
		ROS_INFO_STREAM("init controller ....");
		pos_last_.x = initial_pose_.pose.pose.position.x;
		pos_last_.y = initial_pose_.pose.pose.position.y;
		pos_last_.z = initial_pose_.pose.pose.position.z;
		pos_setpoint_ = pos_last_;

		GetEulerAnglesFromQuaternion(initial_pose_.pose.pose.orientation,euler_last_);
		att_setpoint_ = euler_last_;

		pos_velocity_setpoint_.x = 0;
		pos_velocity_setpoint_.y = 0;
		pos_velocity_setpoint_.z = 0;

		att_velocity_setpoint_.x = 0;
		att_velocity_setpoint_.y = 0;
		att_velocity_setpoint_.z = 0;

		SetAndPubTrajectoryPoint();
		timer_.start();//油门首次推过中点，开始发送第一条指令，并启动定时器。
		std::cout << "has started?: " << timer_.hasStarted()<<std::endl;
		// std::cout << "should have started" << std::endl;
	}

}

/*根据四元数计算欧拉角*/
void RcCommandProcessingNode::GetEulerAnglesFromQuaternion(const geometry_msgs::Quaternion& q, geometry_msgs::Vector3& euler_angles){
    euler_angles.x = std::atan2(2.0 * (q.w * q.x + q.y * q.z),
                           1.0 - 2.0 * (q.x * q.x + q.y * q.y));
	euler_angles.y = 2 * std::atan2(std::sqrt(1 + 2 * (q.w * q.y - q.x * q.z)), 
            				std::sqrt(1 - 2 * (q.w * q.y - q.x * q.z))) - M_PI / 2;
    euler_angles.z = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
							1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}//理论上求出的角度范围是(-pi,pi]

/*从joystick_value确定velocity*/
void RcCommandProcessingNode::MapJoystickToVel(){
	double vel_factor[]={0.0, 0.0, 0.0, 0.0};
	for(int i = 0; i < 4;i++){
		if((float)(abs(joystick_value_[i]-ch_config_.channels[i].mid))>15){//joystick 2 的阈值是15还是25，注意一下，会影响到摇杆的灵敏度。
			vel_factor[i]= - ((float)(joystick_value_[i]-ch_config_.channels[i].mid)/
					(float)(ch_config_.channels[i].max - ch_config_.channels[i].mid));//channel i
		}
	}
	if(position_or_attitude_){//position mode
		twist_.v[0] = vel_factor[1] * twist_.v_max[0];//vx
		twist_.v[1] = vel_factor[3] * twist_.v_max[1];//vy
		twist_.v[2] = vel_factor[2] * twist_.v_max[2];//vz
		twist_.omega[2] = vel_factor[0] * twist_.omega_max[2];//omega_z
	} else{//attitude mode
		twist_.omega[0] = vel_factor[3] * twist_.omega_max[0];//omega_x
		twist_.omega[1] = vel_factor[1] * twist_.omega_max[1];//omega_y
		twist_.omega[2] = vel_factor[0] * twist_.omega_max[2];//omega_z
		twist_.v[2] = vel_factor[2] * twist_.v_max[2];//vz

		euler_vel_[0] = twist_.omega[0] + 
						sin(euler_last_.x) * tan(euler_last_.y) * twist_.omega[1] + 
						cos(euler_last_.x) * tan(euler_last_.y) * twist_.omega[2];
		euler_vel_[1] = cos(euler_last_.x) * twist_.omega[1] -
						sin(euler_last_.x) * twist_.omega[2];
		euler_vel_[2] = sin(euler_last_.x)/cos(euler_last_.y) * twist_.omega[1] + 
						cos(euler_last_.x)/cos(euler_last_.y) * twist_.omega[2];
	}

}

/*计算待发布的轨迹点指令*/
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

/*生成并发布轨迹点指令*/
void RcCommandProcessingNode::SetAndPubTrajectoryPoint(){
	trajectory_point_msg_.points[0].transforms[0].translation = pos_setpoint_;
	trajectory_point_msg_.points[0].transforms[0].rotation = tf::createQuaternionMsgFromRollPitchYaw(att_setpoint_.x, att_setpoint_.y, att_setpoint_.z);
	trajectory_point_msg_.points[0].velocities[0].linear = pos_velocity_setpoint_;
	trajectory_point_msg_.points[0].velocities[0].angular = att_velocity_setpoint_;

	trajectory_point_pub_.publish(trajectory_point_msg_);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "rc_command_processing_node");
	ros::NodeHandle nh;
	RcCommandProcessingNode RcCommandProcessor(nh);
	ros::spin();
	return 0;
}