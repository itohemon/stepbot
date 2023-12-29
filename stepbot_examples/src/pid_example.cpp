#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace pid_example
{
class PidExample : public rclcpp::Node
{
public:
    PidExample(rclcpp::NodeOptions options);
    ~PidExample();
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_twist_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

    void onTimerElapsed();
    void onOdomSubscribed(nav_msgs::msg::Odometry::SharedPtr odom);

    rclcpp::TimerBase::SharedPtr timer_;

    double Kp;
    double Kd;
    double Ki;

    double p_error;
    double i_error;
    double d_error;
    double prev_diff;

    double goal_x;
    double goal_y;
    double goal_z;

    double curr_x;
    double curr_y;
    double curr_z;

    double init_x;
    double init_y;
    double init_z;
};

PidExample::PidExample(rclcpp::NodeOptions options) : Node("PidExample", options)
{
    using std::placeholders::_1;

    /** サブスクライバ初期化 **/
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>("odom", 10,
        std::bind(&PidExample::onOdomSubscribed, this, _1));

    /** パブリッシャ初期化 **/
    pub_twist_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = create_wall_timer(std::chrono::milliseconds(1000/30),
        std::bind(&PidExample::onTimerElapsed, this));

    goal_x = 1.0;
    goal_y = 0.0;
    goal_z = 0.0;

    Kp = 2.0;
    Ki = 0.001;
    Kd = 30.0;

    p_error = 0;
    i_error = 0;
    d_error = 0;
    prev_diff = 0;
}

void PidExample::onOdomSubscribed(nav_msgs::msg::Odometry::SharedPtr odom)
{
    static bool init_flg = false;

    curr_x = odom->pose.pose.position.x;
    curr_y = odom->pose.pose.position.y;
    curr_z = odom->pose.pose.position.z;

    if (!init_flg) {
        init_x = curr_x;
        init_y = curr_y;
        init_z = curr_z;
    }

    RCLCPP_DEBUG(this->get_logger(), "(x, y, z) = (%lf, %lf, %lf)", curr_x, curr_y, curr_z);
}


PidExample::~PidExample()
{

}

void PidExample::onTimerElapsed()
{
    auto msg = std::make_unique<geometry_msgs::msg::Twist>();
    static bool flg = false;

    double diff = goal_x - curr_x;

    if (!flg) {
        flg = true;
        d_error = diff;
        prev_diff = diff;
    } else {
        d_error = diff - prev_diff;
        prev_diff = diff;
    }

    p_error = diff;
    i_error += diff;

    msg->linear.x = p_error * Kp + i_error * Ki + d_error * Kd;
    RCLCPP_INFO(this->get_logger(), "Twist_x: %lf", msg->linear.x);

    pub_twist_->publish(std::move(msg));
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pid_example::PidExample)

