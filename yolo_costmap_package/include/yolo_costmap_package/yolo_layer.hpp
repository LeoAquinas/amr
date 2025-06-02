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
  struct CellData {
    unsigned int x, y;
    unsigned int src_x, src_y;
    CellData(unsigned int x, unsigned int y, unsigned int src_x, unsigned int src_y)
    : x(x), y(y), src_x(src_x), src_y(src_y) {}
  };
  std::vector<bool> seen_;
  std::vector<std::vector<CellData>> inflation_cells_;
  std::vector<std::vector<unsigned char>> cached_cost_matrix_;
  double cached_cell_inflation_radius_;
  int cell_inflation_radius_;
  int cache_length_;
  void computeCaches();
  void enqueue(
    int x, int y,
    unsigned int src_x, unsigned int src_y,
    unsigned int size_x);

  rclcpp::Time last_callback_time_;
  double min_callback_interval_ = 0.2;  // seconds, e.g., 5 Hz max
  rclcpp::Clock::SharedPtr clock_;

  unsigned int size_x;
  unsigned int size_y;
  double resolution;
  int    inscribed_cells;
  int    inflation_cells;


  // === Parameters ===
  bool enabled_;
  double inscribed_radius_;
  double avoidance_radius_;
  double decay_time_;
  std::string sub_topic_;
  std::string global_frame_;
  double resolution_;  // costmap resolution

  // === TF Buffer & Listener ===
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  geometry_msgs::msg::TransformStamped map_to_odom_;

  // === Costmap Related ===
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  bool need_recalculation_{false};
  bool dirty_{false};
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

  // === Detection and Processing ===
  struct Detection {
    double x;
    double y;
    rclcpp::Time stamp;

    Detection(double x_, double y_, rclcpp::Time stamp_)
    : x(x_), y(y_), stamp(stamp_) {}
  };

  std::vector<Detection> detections_;
  std::mutex detection_mutex_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // === Visualization ===
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr detection_pub_;
  visualization_msgs::msg::MarkerArray m_array_;
  int last_marker_count_{0};

  // === Optional (if needed later) ===
  // std::vector<Detection> persistent_detections_;
  // rclcpp::Time last_detection_time_;
  // bool first_update_{true};
  unsigned char computeCost(double distance) const;
  double merge_threshold_;
  size_t last_detection_count_;
  std::vector<std::pair<unsigned int, unsigned int>> last_inflated_cells_;


};

}  // namespace yolo_costmap_package

#endif  // YOLO_LAYER_HPP_