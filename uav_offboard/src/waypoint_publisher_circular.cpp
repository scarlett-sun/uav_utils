#include <fstream>
#include <iostream>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
// #include <mav_msgs/State.h>

#include <tf/tf.h>


class ThirdPolynomial{
public:
  double pos;
  double vel;
private:
  double a;//3, t^3
  double b;//2, t^2
  double c;//1, t^1
  double d;//0, t^0
  bool coeff_set = false;

public:  
  void setCoeff(const double& current_val,const double& desired_val, const double& t){
  d = current_val;
  c = 0;
  b = 3*(desired_val - d) / (t*t);
  a = - 2 * b/3.0/t;
  coeff_set = true;
  }
  // void setCoeff(const Eigen::Vector3d& current_vector, const Eigen::Vector3d& desired_vec, const double& t){

  // }
  void calPos(const double& t){
    if(coeff_set){
      pos = a*t*t*t+ b *t*t + c *t + d;
    }
    else{
      pos =0;
      ROS_ERROR("Coefficient not set!");
    }
    // std::cout << a << " " << b << " " << c << " " << d << std::endl;
  }

  void calVel(const double& t){
    if(coeff_set){
      vel = 3*a*t*t + 2*b*t + c;
    }
    else{
      vel = 0;
      ROS_ERROR("Coefficient not set!");
    }
  }
  double getPos(){
    return pos;
  }
};


class CircularTrajectory{
  double R;
  double T;
  double x0;
  double y0;
  double x;
  double vx;
  double y;
  double vy;
  bool coeff_set = false;
 public:
  void setCoeff(const double& R_d, const double& delay,const double& x0_d, const double& y0_d, const double& loop_number_d){
    R = R_d;
    T = delay/loop_number_d;//total_time/loop_number = loop_time
    x0 = x0_d;
    y0 = y0_d;
    coeff_set = true;
  }
  void calValue(const double& t){
    if(coeff_set){
      x = R*std::cos(2*M_PI/T*t) - R + x0;
      y = R*std::sin(2*M_PI/T*t) + y0;
      vx = -R * std::sin(2*M_PI/T*t)*2*M_PI/T;
      vy = R * std::cos(2*M_PI/T*t)*2*M_PI/T;
    }
    else{
      ROS_ERROR("Coefficient not set!");
    }
  }
  double getX(){return x;}
  double getY(){return y;}
  double getVx(){return vx;}
  double getVy(){return vy;}
};


static const int64_t kNanoSecondsInSecond = 1000000000;//ns
static const double kDeltaT = 0.01;//s
static const float DEG_2_RAD = M_PI / 180.0;

int main(int argc, char** argv) {
  ros::init(argc, argv, "waypoint_publisher_circular");
  ros::NodeHandle nh;
  ros::Publisher wp_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
  ROS_INFO("Started waypoint_publisher_circular.");

  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);

  double delay;

  ROS_INFO("radius: %f", std::stof(args.at(1)));
  ROS_INFO("current x: %f",std::stof(args.at(2)));
  ROS_INFO("current y: %f",std::stof(args.at(3)));
  ROS_INFO("current z: %f",std::stof(args.at(4)));
  ROS_INFO("desired z: %f",std::stof(args.at(5)));
  ROS_INFO("current pitch: %f",std::stof(args.at(6)));
  ROS_INFO("desired pitch: %f",std::stof(args.at(7))); 
  ROS_INFO("current yaw: %f",std::stof(args.at(8)));
  ROS_INFO("desired yaw: %f",std::stof(args.at(9)));  

  ROS_INFO("desired loop number: %f",std::stof(args.at(10)));
  ROS_INFO("delay: %f", std::stof(args.at(11)));//duration

  if (args.size() == 11) {
    delay = 1.0;
  } else if (args.size() == 12) {
    delay = std::stof(args.at(11));
  } else {
    ROS_ERROR("Usage: waypoint_publisher <namespace> <radius_d> <x_sen> <y_sen> <z_sen> <z_d> <pitch_sen> <pitch_d> <yaw_sen> <yaw_d> <loop_d> [<motion duration>]\n");
    return -1;
  }
// rosrun <package_name> <node_name> __ns:=/xxx 2 0 0 1 1 0 0 0 360 1 10
//-------------------------------------------------------------------------------------
  double desired_radius = std::stof(args.at(1));
  double current_x = std::stof(args.at(2));
  double current_y = std::stof(args.at(3));
  double current_z = std::stof(args.at(4));
  double desired_loop_number = std::stof(args.at(10));

  double desired_z = std::stof(args.at(5));
  double current_pitch = std::stof(args.at(6)) * DEG_2_RAD;
  double desired_pitch = std::stof(args.at(7)) * DEG_2_RAD;
  double current_yaw = std::stof(args.at(8)) * DEG_2_RAD;
  double desired_yaw = std::stof(args.at(9)) * DEG_2_RAD;
//-------------------------------------------------------------------------------------
  ROS_INFO("About to construct coefficient");
  ROS_INFO("Wait for 3 seconds");
  ros::Duration(3).sleep();//问题症结所在，不知道为什么会这样，这里会卡住

  CircularTrajectory circular_trajectory;
  circular_trajectory.setCoeff(desired_radius,delay,current_x,current_y,desired_loop_number);

  ThirdPolynomial pitch_com;
  pitch_com.setCoeff(current_pitch,desired_pitch,delay);
  ThirdPolynomial yaw_com;
  yaw_com.setCoeff(current_yaw,desired_yaw,delay);
  ThirdPolynomial position_z_com;
  position_z_com.setCoeff(current_z, desired_z, delay);


  int points_number = delay * 100;
  ROS_INFO("Start publishing waypoints.");
//--------------------------------------------------------------------------
  trajectory_msgs::MultiDOFJointTrajectoryPtr msg(new trajectory_msgs::MultiDOFJointTrajectory);
  msg->header.stamp = ros::Time::now();
  msg->points.resize(points_number);
  msg->joint_names.push_back("base_link");
  int64_t time_from_start_ns = 0;
  geometry_msgs::Quaternion q;
  mav_msgs::EigenTrajectoryPoint trajectory_point;
  for (size_t i = 0; i < points_number; ++i) {
    trajectory_point.time_from_start_ns = time_from_start_ns;
    time_from_start_ns += static_cast<int64_t>(kDeltaT * kNanoSecondsInSecond);
    double time = double(time_from_start_ns) / kNanoSecondsInSecond;
    circular_trajectory.calValue(time);
    position_z_com.calPos(time);  position_z_com.calVel(time);
    yaw_com.calPos(time);
    pitch_com.calPos(time);
    trajectory_point.position_W = Eigen::Vector3d(circular_trajectory.getX(),circular_trajectory.getY(),position_z_com.pos);
    trajectory_point.velocity_W = Eigen::Vector3d(circular_trajectory.getVx(),circular_trajectory.getVy(),position_z_com.vel);
    mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &msg->points[i]);
    q=tf::createQuaternionMsgFromRollPitchYaw(0,pitch_com.pos,yaw_com.pos);
    msg->points[i].transforms.at(0).rotation = q;
  }

  ros::Duration(1).sleep();//问题症结所在，不知道为什么会这样，这里会卡住
  //但是对面应该是收到了消息，不然怎么激活控制器呢？
  wp_pub.publish(msg);

  ros::spinOnce();
  ros::shutdown();

  return 0;
}

