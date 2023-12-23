#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include "stepbot_node/stepbot_status.hpp"

#define WHEEL_RAD 0.035   // ホイール半径
#define WHEEL_SEP 0.186   // トレッド

using namespace std::chrono_literals;
using std::placeholders::_1;

StepbotStatus::StepbotStatus() : Node("stepbot_status")
{
  publish_tf_ = true;
  this->get_parameter("publish_tf", publish_tf_);
  
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

  jointstate_pub_
    = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  
  wheelstate_sub_ 
    = this->create_subscription<std_msgs::msg::Float32MultiArray>("wheel_state", 10,
								  std::bind(&StepbotStatus::wheelStateCb, this, _1));
  
  odom_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  
  joint_state_l_ = "wheel_left_joint";
  joint_state_r_ = "wheel_right_joint";
}

void StepbotStatus::wheelStateCb(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  auto odom_msg = nav_msgs::msg::Odometry();
  auto state_msg = sensor_msgs::msg::JointState();

  if (msg->data.size() < 1) return;

  current_time_ = this->get_clock()->now();

  double vx     = msg->data[0]; // 車体X方向速度[m/s]
  double vy     = msg->data[1]; // 車体Y方向速度[m/s]
  double vth    = msg->data[2]; // 車体角速度[rad/s]
  double wL     = msg->data[3]; // 左車軸回転角速度[rad/s]
  double wR     = msg->data[4]; // 右車軸回転角速度[rad/s]
  double jointL = msg->data[5]; // 左車輪角度
  double jointR = msg->data[6]; // 右車輪角度
  RCLCPP_DEBUG(this->get_logger(), 
    "Subscribe vx: %f, vth: %f, wL: %lf  wR: %lf", vx, vth, wL, wR);

  double dt = current_time_.seconds() - last_time_.seconds();
  double delta_x = (vx * cos(th_) - vy * sin(th_)) * dt;
  double delta_y = (vx * sin(th_) + vy * cos(th_)) * dt;
  double delta_th = vth * dt;
  
  // [Reference]
  // http://wiki.ros.org/ja/navigation/Tutorials/RobotSetup/Odom
  x_ += delta_x;
  y_ += delta_y;
  th_ += delta_th;

  //since all odometry is 6DOF we'll need a quaternion created from yaw
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, th_);

  //first, we'll publish the transform over tf
  odom_msg.header.stamp = current_time_;
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";

  //set the position
  odom_msg.pose.pose.position.x = x_;
  odom_msg.pose.pose.position.y = y_;
  odom_msg.pose.pose.position.z = 0.0;

  odom_msg.pose.pose.orientation.x = q.x();
  odom_msg.pose.pose.orientation.y = q.y();
  odom_msg.pose.pose.orientation.z = q.z();
  odom_msg.pose.pose.orientation.w = q.w();

  //set the velocity
  odom_msg.twist.twist.linear.x = vx;
  odom_msg.twist.twist.linear.y = vy;
  odom_msg.twist.twist.angular.z = vth;

  odom_pub_->publish(odom_msg);

  // JointStatesを作ってパブリッシュする
  state_msg.header.stamp = current_time_;
  state_msg.name.resize(2);
  state_msg.name[0] = joint_state_l_.c_str();
  state_msg.name[1] = joint_state_r_.c_str();
  state_msg.position.resize(2);
  state_msg.position[0] = jointL;
  state_msg.position[1] = jointR;
  
  jointstate_pub_->publish(state_msg);


  // odomフレーム(TF)をブロードキャストする
  geometry_msgs::msg::TransformStamped odom_tf;
  
  odom_tf.transform.translation.x = odom_msg.pose.pose.position.x;
  odom_tf.transform.translation.y = odom_msg.pose.pose.position.y;
  odom_tf.transform.translation.z = odom_msg.pose.pose.position.z;
  odom_tf.transform.rotation      = odom_msg.pose.pose.orientation;
  
  odom_tf.header.frame_id = odom_msg.header.frame_id;
  odom_tf.header.stamp    = odom_msg.header.stamp;
  odom_tf.child_frame_id  = odom_msg.child_frame_id;

  if (publish_tf_) {
    odom_tf_broadcaster_->sendTransform(odom_tf);
  }
  last_time_ = current_time_;
}

int main(int argc,  char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StepbotStatus>());
  rclcpp::shutdown();

  return 0;
}
