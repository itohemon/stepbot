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

    double x;
    double y;
    double z;
};

PidExample::PidExample(rclcpp::NodeOptions options) : Node("PidExample", options)
{
    using std::placeholders::_1;

    /** サブスクライバ初期化 **/
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>("odom", 10,
        std::bind(&PidExample::onOdomSubscribed, this, _1));

    /** パブリッシャ初期化 **/
    pub_twist_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    timer_ = create_wall_timer(std::chrono::milliseconds(1000/30),
        std::bind(&PidExample::onTimerElapsed, this));
}

void PidExample::onOdomSubscribed(nav_msgs::msg::Odometry::SharedPtr odom)
{
    x = odom->pose.pose.position.x;
    y = odom->pose.pose.position.y;
    z = odom->pose.pose.position.z;

    RCLCPP_INFO(this->get_logger(), "(x, y, z) = (%lf, %lf, %lf)", x, y, z);
}


PidExample::~PidExample()
{

}

void PidExample::onTimerElapsed()
{
    auto msg = std::make_unique<geometry_msgs::msg::Twist>();

    pub_twist_->publish(std::move(msg));
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pid_example::PidExample)

