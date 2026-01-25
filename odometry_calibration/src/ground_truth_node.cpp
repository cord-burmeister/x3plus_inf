#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h> 
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
 #include <tf2/LinearMath/Quaternion.h>
 #include <tf2/LinearMath/Matrix3x3.h>



class GroundTruthNode : public rclcpp::Node
{
public:
    GroundTruthNode() : Node("ground_truth_node"),
                        tf_buffer_(this->get_clock()),
                        tf_listener_(tf_buffer_)
    {
        detections_sub_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
            "/detections", 10,
            std::bind(&GroundTruthNode::detectionsCallback, this, std::placeholders::_1));

        gt_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ground_truth/pose", 10);

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        RCLCPP_INFO(this->get_logger(), "Ground truth node started");
    }

private:

    void publishTransform(const geometry_msgs::msg::Pose &pose) 
    { 
        geometry_msgs::msg::TransformStamped tf; 
        tf.header.stamp = this->now(); 
        tf.header.frame_id = "world"; // parent 
        tf.child_frame_id = "base_footprint"; // child 
        tf.transform.translation.x = pose.position.x; 
        tf.transform.translation.y = pose.position.y; 
        tf.transform.translation.z = pose.position.z; 
        tf.transform.rotation = pose.orientation; 
        tf_broadcaster_->sendTransform(tf); 
    }

    geometry_msgs::msg::TransformStamped planarizeTransform(
        const geometry_msgs::msg::TransformStamped& input,
        bool zero_z = true)
    {
        geometry_msgs::msg::TransformStamped out = input;

        // --- 1. Extract yaw from quaternion ---
        tf2::Quaternion q(
            input.transform.rotation.x,
            input.transform.rotation.y,
            input.transform.rotation.z,
            input.transform.rotation.w
        );

        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // --- 2. Rebuild a pure yaw quaternion ---
        tf2::Quaternion q_planar;
        q_planar.setRPY(0.0, 0.0, yaw);
        q_planar.normalize();

        out.transform.rotation.x = q_planar.x();
        out.transform.rotation.y = q_planar.y();
        out.transform.rotation.z = q_planar.z();
        out.transform.rotation.w = q_planar.w();

        // --- 3. Optionally zero out Z translation ---
        if (zero_z) {
            out.transform.translation.z = 0.0;
        }

        return out;
    }


    void detectionsCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
    {
        geometry_msgs::msg::PoseStamped cam_to_worldtag;
        geometry_msgs::msg::PoseStamped cam_to_robottag;

        bool found_worldtag = false;
        bool found_robottag = false;

        // Extract tag poses from detection array
        for (auto &det : msg->detections)
        {
            if (det.id == 0) {  // world reference tag
                found_worldtag = true;
            }
            if (det.id == 1) {  // robot tag
                found_robottag = true;
            }
        }

        if (!found_worldtag || !found_robottag)
            return;

        try
        {
            // Static transforms from URDF or static publishers
            geometry_msgs::msg::TransformStamped world_to_worldtag =
                tf_buffer_.lookupTransform("world", "world_tag", tf2::TimePointZero);

            geometry_msgs::msg::TransformStamped robottag_to_base =
                tf_buffer_.lookupTransform("robot_tag", "base_footprint", tf2::TimePointZero);

            geometry_msgs::msg::TransformStamped  cam_to_worldtag = tf_buffer_.lookupTransform(
                "c920_camera_link", "tag36h11:0", tf2::TimePointZero);
            //cam_to_worldtag = planarizeTransform(cam_to_worldtag, true);

            geometry_msgs::msg::TransformStamped  cam_to_robottag = tf_buffer_.lookupTransform(
                "c920_camera_link", "tag36h11:1", tf2::TimePointZero);
            //cam_to_robottag = planarizeTransform(cam_to_robottag, false);

            // Convert AprilTag poses into transforms
            geometry_msgs::msg::TransformStamped cam_to_worldtag_tf;
            cam_to_worldtag_tf.header = cam_to_worldtag.header;
            cam_to_worldtag_tf.child_frame_id = "world_tag";
            cam_to_worldtag_tf.transform.translation.x = cam_to_worldtag.transform.translation.x;
            cam_to_worldtag_tf.transform.translation.y = cam_to_worldtag.transform.translation.y;
            cam_to_worldtag_tf.transform.translation.z = cam_to_worldtag.transform.translation.z;
            cam_to_worldtag_tf.transform.rotation = cam_to_worldtag.transform.rotation;

            geometry_msgs::msg::TransformStamped cam_to_robottag_tf;
            cam_to_robottag_tf.header = cam_to_robottag.header;
            cam_to_robottag_tf.child_frame_id = "robot_tag";
            cam_to_robottag_tf.transform.translation.x = cam_to_robottag.transform.translation.x;
            cam_to_robottag_tf.transform.translation.y = cam_to_robottag.transform.translation.y;
            cam_to_robottag_tf.transform.translation.z = cam_to_robottag.transform.translation.z;
            cam_to_robottag_tf.transform.rotation = cam_to_robottag.transform.rotation;
            //cam_to_robottag_tf = planarizeTransform(cam_to_robottag_tf, false);

            // Compute world → base_footprint
            tf2::Transform T_world_worldtag, T_worldtag_camera, T_camera_robottag, T_robottag_base;

            tf2::fromMsg(world_to_worldtag.transform, T_world_worldtag);
            tf2::fromMsg(cam_to_worldtag_tf.transform, T_worldtag_camera);
            tf2::fromMsg(cam_to_robottag_tf.transform, T_camera_robottag);
            tf2::fromMsg(robottag_to_base.transform, T_robottag_base);

            tf2::Transform T_world_base =
                T_world_worldtag * T_worldtag_camera.inverse() * T_camera_robottag * T_robottag_base;

            // Publish pose
            geometry_msgs::msg::PoseStamped out;
            out.header.stamp = this->now();
            out.header.frame_id = "world";
            // Convert tf2::Transform to geometry_msgs::msg::Pose
            tf2::Vector3 origin = T_world_base.getOrigin();
            tf2::Quaternion rot = T_world_base.getRotation();
            out.pose.position.x = origin.x();
            out.pose.position.y = origin.y();
            out.pose.position.z = origin.z();
            // TODO Check correctness of orientation assignment
            // out.pose.orientation.x = rot.x();
            // out.pose.orientation.y = rot.y();
            out.pose.orientation.x = 0.0;
            out.pose.orientation.y = 0.0;
            out.pose.orientation.z = rot.z();
            out.pose.orientation.w = rot.w();

            gt_pub_->publish(out);

            publishTransform(out.pose);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF error: %s", ex.what());
        }
    }

    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detections_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr gt_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GroundTruthNode>());
    rclcpp::shutdown();
    return 0;
}
