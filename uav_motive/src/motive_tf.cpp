#include <Eigen/Eigen>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>


void getEulerAnglesFromQuaternionNew(const Eigen::Quaternion<double>& q, Eigen::Vector3d& euler_angles){
    
    euler_angles << std::atan2(2.0 * (q.w() * q.x() + q.y() * q.z()),
                           1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y())),
        2 * std::atan2(std::sqrt(1 + 2 * (q.w() * q.y() - q.x() * q.z())), 
            std::sqrt(1 - 2 * (q.w() * q.y() - q.x() * q.z()))) - M_PI / 2,
        std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
              1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    // euler_angles << std::atan2(2.0 * (q.w * q.x + q.y * q.z),
    //                     1.0 - 2.0 * (q.x * q.x + q.y * q.y)),
    // 2 * std::atan2(std::sqrt(1 + 2 * (q.w * q.y - q.x * q.z)), 
    //     std::sqrt(1 - 2 * (q.w() * q.y() - q.x() * q.z()))) - M_PI / 2,
    // std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
    //         1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}


void motive_cb(const geometry_msgs::PoseStampedConstPtr& msg){
    Eigen::Quaterniond quaternion(msg->pose.orientation.w, 
                                  msg->pose.orientation.x, 
                                  msg->pose.orientation.y,
                                  msg->pose.orientation.z);
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix=quaternion.toRotationMatrix();
    Eigen::Matrix3d Re_0;
    Re_0 << -1,0,0,
              0,0,1,
              0,1,0;
    Eigen::Matrix3d Re_b = Re_0*rotation_matrix;
    Eigen::Quaterniond new_quaternion(Re_b);
    Eigen::Vector3d eulerAngle;
    getEulerAnglesFromQuaternionNew(new_quaternion,eulerAngle);
    ROS_INFO("ENU euler: %s",eulerAngle.x()," ",eulerAngle.y()," ",eulerAngle.z());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "motive_tf");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/RigidBody01/pose",10,motive_cb);
  ros::spin();//循环读取接收的数据，并调用回调函数处理


  return 0;
}