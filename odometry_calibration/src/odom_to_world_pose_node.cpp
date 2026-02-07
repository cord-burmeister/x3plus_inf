#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class OdomToWorldPoseNode : public rclcpp::Node
{
public:
    OdomToWorldPoseNode()
    : Node("odom_to_world_pose_node")
    {
        using std::placeholders::_1;

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/wheel/odometry",
            rclcpp::SensorDataQoS(),
            std::bind(&OdomToWorldPoseNode::odomCallback, this, _1));

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/world_pose",
            10);

        RCLCPP_INFO(this->get_logger(), "Odom → World Pose node started");
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        geometry_msgs::msg::PoseStamped pose_msg;

        // Copy timestamp
        pose_msg.header.stamp = msg->header.stamp;

        // Set target frame
        pose_msg.header.frame_id = "world";

        // Copy pose directly (assuming world == odom for calibration)
        pose_msg.pose = msg->pose.pose;

        pose_pub_->publish(pose_msg);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomToWorldPoseNode>());
    rclcpp::shutdown();
    return 0;
}
