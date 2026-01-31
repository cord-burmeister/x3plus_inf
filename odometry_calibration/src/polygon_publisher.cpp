#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"

class PolygonPublisher : public rclcpp::Node
{
public:
  PolygonPublisher()
  : Node("polygon_publisher")
  {
    // parameters: publish period (ms) and frame id for the marker
    int publish_period_ms = this->declare_parameter<int>("publish_period_ms", 500);
    frame_id_ = this->declare_parameter<std::string>("frame_id", "world");

    world_offset_ = this->declare_parameter<double>("world_offset", 0.3);
    camera_height_ = this->declare_parameter<double>("camera_height", 1.500);
    camera_angle_ = this->declare_parameter<double>("camera_angle", 30);
    robot_height_ = this->declare_parameter<double>("robot_height", 0.4);
    tag_size_ = this->declare_parameter<double>("tag_size", 0.1);
    fovh_ = this->declare_parameter<double>("fovh", 70.1);  // 70.1 degrees
    fovv_ = this->declare_parameter<double>("fovv", 43.3);  // 43.3 degrees

    calculatePolygons();

    publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("calibration_area", 10);
    robot_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("robot_area", 10);
    timer_ = this->create_wall_timer(
    std::chrono::milliseconds(publish_period_ms),
    std::bind(&PolygonPublisher::publish_polygon, this));
  }

private:

void calculatePolygons()
  {

    // distance from camera origing to view of field to ground
    double dgh0 = camera_height_ * cos((camera_angle_ - fovv_ / 2.0) * M_PI / 180.0);
    double dgh1 = camera_height_ * cos((camera_angle_ + fovv_ / 2.0) * M_PI / 180.0);
    double drh0 = (camera_height_ - robot_height_) * cos((camera_angle_ - fovv_ / 2.0) * M_PI / 180.0);
    double drh1 = (camera_height_ - robot_height_) * cos((camera_angle_ + fovv_ / 2.0) * M_PI / 180.0);
    
    double ggh0 = camera_height_ * tan((camera_angle_ - fovv_ / 2.0) * M_PI / 180.0);
    double ggh1 = camera_height_ * tan((camera_angle_ + fovv_ / 2.0) * M_PI / 180.0);
    double grh0 = (camera_height_ - robot_height_) * tan((camera_angle_ - fovv_ / 2.0) * M_PI / 180.0);
    double grh1 = (camera_height_ - robot_height_) * tan((camera_angle_ + fovv_ / 2.0) * M_PI / 180.0);
 
    double wgh0 = dgh0 * tan(fovh_ * M_PI / 180.0) / 2.0;
    double wgh1 = dgh1 * tan(fovh_ * M_PI / 180.0) / 2.0;
    double wrh0 = drh0 * tan(fovh_ * M_PI / 180.0) / 2.0;
    double wrh1 = drh1 * tan(fovh_ * M_PI / 180.0)  / 2.0;
    
    groundPolygon_.clear();
    groundPolygon_.emplace_back(ggh0 - world_offset_, -wgh1);
    groundPolygon_.emplace_back(ggh0 - world_offset_, wgh1);
    groundPolygon_.emplace_back(ggh1 - world_offset_, wgh0);
    groundPolygon_.emplace_back(ggh1 - world_offset_, -wgh0);
    groundPolygon_.emplace_back(ggh0 - world_offset_, -wgh1);  // Close


    robotPolygon_.clear();
    robotPolygon_.emplace_back(grh0 - world_offset_, -wrh1);
    robotPolygon_.emplace_back(grh0 - world_offset_, wrh1);
    robotPolygon_.emplace_back(grh1 - world_offset_, wrh0);
    robotPolygon_.emplace_back(grh1 - world_offset_, -wrh0);
    robotPolygon_.emplace_back(grh0 - world_offset_, -wrh1);  // Close
  }

  void publish_polygon()
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = this->now();
    marker.ns = "polygon";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 0.02;  // line width

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

 
    for (auto & p : groundPolygon_) {
      geometry_msgs::msg::Point point;
      point.x = p.first;
      point.y = p.second;
      point.z = 0.0;
      marker.points.push_back(point);
    }

    publisher_->publish(marker);

     visualization_msgs::msg::Marker robot_marker;
    robot_marker.header.frame_id = frame_id_;
    robot_marker.header.stamp = this->now();
    robot_marker.ns = "polygon";
    robot_marker.id = 0;
    robot_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    robot_marker.action = visualization_msgs::msg::Marker::ADD;

    robot_marker.scale.x = 0.02;  // line width

    robot_marker.color.r = 0.0f;
    robot_marker.color.g = 0.0f;
    robot_marker.color.b = 1.0f;
    robot_marker.color.a = 1.0f;

 
    for (auto & p : robotPolygon_) {
      geometry_msgs::msg::Point point;
      point.x = p.first;
      point.y = p.second;
      point.z = robot_height_;
      robot_marker.points.push_back(point);
    }

    robot_publisher_->publish(robot_marker);
  }

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr robot_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string frame_id_;
  double world_offset_ = 0.3;
  double camera_height_ = 1.5;
  double camera_angle_ = 15;
  double robot_height_ = 0.4;
  double tag_size_ = 0.1;
  double fovh_ = 70.1;  // 70.1 degrees
  double fovv_ = 43.3;

  std::vector<std::pair<double, double>> groundPolygon_=  std::vector<std::pair<double, double>> ();
  std::vector<std::pair<double, double>> robotPolygon_=  std::vector<std::pair<double, double>> ();
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PolygonPublisher>());
  rclcpp::shutdown();
  return 0;
}
