#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

// パラメータデフォルト値
#define HZ 20
#define WHEEL_ACC_X  0.64
#define WHEEL_ACC_Y  0.0
#define WHEEL_ACC_TH 6.0
#define TWIST_TIMEOUT 0.1

using namespace std::placeholders;

class RampedTeleop : public rclcpp::Node
{
public:
  RampedTeleop(rclcpp::NodeOptions options = rclcpp::NodeOptions());
  ~RampedTeleop(){};
  
private:
  geometry_msgs::msg::Twist target_twist_;
  geometry_msgs::msg::Twist current_twist_;
  rclcpp::Time last_twist_time_;
  rclcpp::TimerBase::SharedPtr timer_;

  double acc_x_;                // X方向加速度
  double acc_y_;                // Y方向加速度
  double acc_th_;               // Theta加速度
  int hz_;                      // 出力レート
  double timeout_;              // twistの有効時間

  void setParams();
  void onTimerElapsed();
  void onTwistSubscribed(const geometry_msgs::msg::Twist _twist);
  double ramped_vel(double _v_prev, double _v_target, double _ramp_rate);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_out_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_vel_in_;
};

RampedTeleop::RampedTeleop(rclcpp::NodeOptions options)
: Node("ramped_teleop", options)
{
  setParams();

  pub_vel_out_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  sub_vel_in_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_raw", 10,
    std::bind(&RampedTeleop::onTwistSubscribed, this, _1));

  timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / hz_),
    std::bind(&RampedTeleop::onTimerElapsed, this));
}

/*
 * パラメータ設定
 */
void RampedTeleop::setParams()
{
  // declared ros2 param
  this->declare_parameter<double>("acc_x",  WHEEL_ACC_X);
  this->declare_parameter<double>("acc_y",  WHEEL_ACC_Y);
  this->declare_parameter<double>("acc_th", WHEEL_ACC_TH);
  this->declare_parameter<int>("publish_rate", HZ);
  this->declare_parameter<double>("timeout", TWIST_TIMEOUT);
  // set ros2 param
  this->get_parameter("acc_x",  acc_x_);
  this->get_parameter("acc_y",  acc_y_);
  this->get_parameter("acc_th", acc_th_);
  this->get_parameter("publish_rate", hz_);
  this->get_parameter("timeout", timeout_);
}

/*
 * Publishする速度を計算
 */
void RampedTeleop::onTimerElapsed()
{
  // timeout_[s]以上twistが送られなかったら停止動作に切り替え
  rclcpp::Time now = this->get_clock()->now();
  double dt = now.seconds() - last_twist_time_.seconds();
  if (dt > timeout_) {
    target_twist_.linear.x  = 0.0;
    target_twist_.linear.y  = 0.0;
    target_twist_.angular.z = 0.0;
  }
  
  // 各方向で加減速を考慮した速度指令を算出
  current_twist_.linear.x  = ramped_vel(current_twist_.linear.x,  target_twist_.linear.x,  acc_x_);
  current_twist_.linear.y  = ramped_vel(current_twist_.linear.y,  target_twist_.linear.y,  acc_y_);
  current_twist_.angular.z = ramped_vel(current_twist_.angular.z, target_twist_.angular.z, acc_th_);

  pub_vel_out_->publish(current_twist_);
}

/*
 * Twistを内部に取り込む
 */
void RampedTeleop::onTwistSubscribed(const geometry_msgs::msg::Twist _twist)
{
  target_twist_ = _twist;
  last_twist_time_ = this->now();
}

/*
 * 加減速を考慮した速度計算を行う
 * プログラミングROS(Oreilly)P.127参照
 * _v_prev: 現在の速度
 * _v_target: 目標速度
 * _ramp_rate: 加速度
 */
double RampedTeleop::ramped_vel(double _v_prev, double _v_target, double _ramp_rate)
{
  double step = _ramp_rate / hz_;
  double sign = 1.0;
  double error = fabs(_v_target - _v_prev);

  if (_v_target < _v_prev) {
    sign = -1.0;
  }

  if (error < step) {
    return _v_target;
  } else {
    return _v_prev + sign * step;
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RampedTeleop>();
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
