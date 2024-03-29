#ifndef INCLUDE_UAV_OFFBOARD_RC_COMMON_H_
#define INCLUDE_UAV_OFFBOARD_RC_COMMON_H_

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

static constexpr float kDefault_Vx_Max = 0.1;
static constexpr float kDefault_Vy_Max = 0.1;
static constexpr float kDefault_Vz_Max = 0.1;
static constexpr float kDefault_OmegaX_Max = 0.1;
static constexpr float kDefault_OmegaY_Max = 0.1;
static constexpr float kDefault_OmegaZ_Max = 0.1;

static constexpr float kDefaultDt = 0.01;

static constexpr int kDefaultMin = 1094;
static constexpr int kDefaultMid = 1514;
static constexpr int kDefaultMax = 1934;


struct Twist {
  Twist()
      : v(0.0, 0.0, 0.0),
        omega(0.0, 0.0, 0.0),
        v_max(kDefault_Vx_Max,kDefault_Vy_Max,kDefault_Vz_Max),
        omega_max(kDefault_OmegaX_Max,kDefault_OmegaY_Max, kDefault_OmegaZ_Max) {};

  Eigen::Vector3d v; // linear velocity in body frame
  Eigen::Vector3d omega; // angular velocity in body frame

  Eigen::Vector3d v_max;
  Eigen::Vector3d omega_max;
};

struct Channel {
  Channel()
      : min(kDefaultMin),
        mid(kDefaultMid),
        max(kDefaultMax) {}
  Channel(const int& _min,const int& _mid,const int& _max)
      : min(_min),
        mid(_mid),
        max(_max) {}

  int min;
  int mid;
  int max;
};

struct ChannelConfiguration {
  ChannelConfiguration() {
    // Rotor configuration of Asctec Firefly.
    // 所以说，如果要用ardrone，下面的内容得修改
    channels.push_back(Channel());//从rotors_description的firefly.xacro，0是逆时针ccw，所以1应该是逆时针
    channels.push_back(Channel());
    channels.push_back(Channel());
    channels.push_back(Channel());
  }
  std::vector<Channel> channels;
};

/*get ros parameter channel configuration*/
inline void GetChannelConfiguration(const ros::NodeHandle& nh,
                                    ChannelConfiguration& channel_configuration) {
  std::map<std::string, double> single_channel;
  std::string channel_configuration_string = "channel_configuration/";
  unsigned int i = 0;
  while (nh.getParam(channel_configuration_string + std::to_string(i), single_channel)) {
    if (i == 0) {
      channel_configuration.channels.clear();
    }
    Channel channel;
    nh.getParam(channel_configuration_string + std::to_string(i) + "/min",
                 channel.min);
    nh.getParam(channel_configuration_string + std::to_string(i) + "/mid",
                 channel.mid);
    nh.getParam(channel_configuration_string + std::to_string(i) + "/max",
                 channel.max);
    channel_configuration.channels.push_back(channel);
    ++i;
  }
}

/*get ros parameter max velocity*/
inline void GetMaxVelocity(const ros::NodeHandle& nh,Twist& twist){
  nh.getParam("linear_velocity_max/x",twist.v_max[0]);
  nh.getParam("linear_velocity_max/y",twist.v_max[1]);
  nh.getParam("linear_velocity_max/z",twist.v_max[2]);
  nh.getParam("angular_velocity_max/x",twist.omega_max[0]);
  nh.getParam("angular_velocity_max/y",twist.omega_max[1]);
  nh.getParam("angular_velocity_max/z",twist.omega_max[2]);
}

/*get ros parameter delta_t*/
inline void GetDeltaT(const ros::NodeHandle& nh, float& delta_t){
  nh.getParam("delta_t",delta_t);
}

/*get ros parameter need_offboard*/
inline void GetMode(const ros::NodeHandle& nh, bool& need_offboard){
  nh.getParam("need_offboard",need_offboard);
}

/*Get euler angles from quaternion*/
inline void GetEulerAnglesFromQuaternion(const geometry_msgs::Quaternion& q, 
                                        geometry_msgs::Vector3& euler_angles){
  euler_angles.x = std::atan2(2.0 * (q.w * q.x + q.y * q.z),
                           1.0 - 2.0 * (q.x * q.x + q.y * q.y));
	euler_angles.y = 2 * std::atan2(std::sqrt(1 + 2 * (q.w * q.y - q.x * q.z)), 
            				std::sqrt(1 - 2 * (q.w * q.y - q.x * q.z))) - M_PI / 2;
  euler_angles.z = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
							1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
#endif