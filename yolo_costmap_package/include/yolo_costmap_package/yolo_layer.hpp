#ifndef YOLO_LAYER_HPP_
#define YOLO_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"


#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include<string>
#include<vector>
#include<mutex>
// #include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace yolo_costmap_package
{

class YoloLayer : public nav2_costmap_2d::Layer
{
public:
  YoloLayer();
  virtual ~YoloLayer();

  virtual void onInitialize() override;
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y) override;
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  virtual void reset() override
  {
    return;
  }

  virtual void onFootprintChanged();

  virtual bool isClearable() override {return true;}

private:
  bool enabled_;
  double avoidance_radius_;
  std::string sub_topic_;

  // Pointcloud subscription
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr detection_pub_;
  visualization_msgs::msg::MarkerArray m_array_;

//   std::vector<geometry_msgs::msg::Point> person_points_;
//   std::vector<geometry_msgs::msg::Point> dog_points_;










  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

  // Indicates that the entire gradient should be recalculated next time.
  bool need_recalculation_;

  // Size of gradient in cells
  int GRADIENT_SIZE = 20;
  // Step of increasing cost per one cell in gradient
  int GRADIENT_FACTOR = 10;


protected:
  // Add these members
  bool dirty_;
  std::mutex detection_mutex_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};  // your private buffer

  int last_marker_count_{0};

  struct Detection {
    double x;
    double y;
    rclcpp::Time stamp;
    
    Detection(double x, double y, rclcpp::Time stamp) 
      : x(x), y(y), stamp(stamp) {}
  };
  
  std::vector<Detection> detections_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string global_frame_;
  double resolution_;

  double detection_min_x_;
double detection_min_y_;
double detection_max_x_;
double detection_max_y_;

rclcpp::Time last_detection_time_;

std::vector<Detection> persistent_detections_;
bool first_update_{true};
};

}  // namespace nav2_gradient_costmap_plugin

#endif  // YOLO_LAYER_HPP_