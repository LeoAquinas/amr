#include "yolo_costmap_package/yolo_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"


#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <cmath>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
//visualization_msgs::msg::Marker


using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::FREE_SPACE;

namespace yolo_costmap_package
{

YoloLayer::YoloLayer()
:
  last_min_x_(-std::numeric_limits<float>::max()),
  last_min_y_(-std::numeric_limits<float>::max()),
  last_max_x_(std::numeric_limits<float>::max()),
  last_max_y_(std::numeric_limits<float>::max()),
  dirty_(false) 
{
}
YoloLayer::~YoloLayer() = default;

// This method is called at the end of plugin initialization.
// It contains ROS parameter(s) declaration and initialization
// of need_recalculation_ variable.
void
YoloLayer::onInitialize()
{
  auto node = node_.lock(); 

  // Declare & get parameters
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("avoidance_radius", rclcpp::ParameterValue(0.5));
  declareParameter("subscription_topic", rclcpp::ParameterValue(std::string("")));


  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "avoidance_radius", avoidance_radius_);
  node->get_parameter(name_ + "." + "subscription_topic", sub_topic_);

  global_frame_ = layered_costmap_->getGlobalFrameID();
  resolution_ = layered_costmap_->getCostmap()->getResolution();

  pointcloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    sub_topic_, 10,
    std::bind(&YoloLayer::pointcloudCallback, this, std::placeholders::_1));

  detection_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "detection/detected_cloud", 1);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  costmap_ = layered_costmap_->getCostmap();  // private buffer for your layer


  need_recalculation_ = false;
  current_ = true;
}

void YoloLayer::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(detection_mutex_);
    
    // Save current detections as persistent
    persistent_detections_ = detections_;
    detections_.clear();

    // Transform cloud to map frame
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer_->lookupTransform(
            global_frame_, 
            msg->header.frame_id,
            tf2::TimePointZero,
            tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(node_.lock()->get_logger(), "TF error: %s", ex.what());
        return;
    }

    // Process point cloud
    sensor_msgs::msg::PointCloud2 transformed_cloud;
    tf2::doTransform(*msg, transformed_cloud, transform);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(transformed_cloud, *cloud);

    auto now = node_.lock()->now();
    for (const auto& point : cloud->points) {
        detections_.emplace_back(point.x, point.y, now);
    }

    last_detection_time_ = now;
    need_recalculation_ = true;
}

// The method is called to ask the plugin: which area of costmap it needs to update.
// Inside this method window bounds are re-calculated if need_recalculation_ is true
// and updated independently on its value.

void YoloLayer::updateBounds(
    double robot_x, double robot_y, double /*robot_yaw*/,
    double* min_x, double* min_y, double* max_x, double* max_y)
{
    std::lock_guard<std::mutex> lock(detection_mutex_);

    // Initialize bounds to extreme values on first update
    if (first_update_) {
        last_min_x_ = *min_x;
        last_min_y_ = *min_y;
        last_max_x_ = *max_x;
        last_max_y_ = *max_y;
        first_update_ = false;
    }

    // Calculate detection bounds
    double det_min_x = std::numeric_limits<double>::max();
    double det_min_y = std::numeric_limits<double>::max();
    double det_max_x = -std::numeric_limits<double>::max();
    double det_max_y = -std::numeric_limits<double>::max();

    for (const auto& det : persistent_detections_) {
        det_min_x = std::min(det_min_x, det.x - avoidance_radius_);
        det_min_y = std::min(det_min_y, det.y - avoidance_radius_);
        det_max_x = std::max(det_max_x, det.x + avoidance_radius_);
        det_max_y = std::max(det_max_y, det.y + avoidance_radius_);
    }

    // Expand bounds to include both new detections and previous area
    *min_x = std::min({det_min_x, last_min_x_, *min_x});
    *min_y = std::min({det_min_y, last_min_y_, *min_y});
    *max_x = std::max({det_max_x, last_max_x_, *max_x});
    *max_y = std::max({det_max_y, last_max_y_, *max_y});

    // Store for next iteration
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
}

// The method is called when footprint was changed.
// Here it just resets need_recalculation_ variable.
void
YoloLayer::onFootprintChanged()
{
  need_recalculation_ = true;

  RCLCPP_DEBUG(rclcpp::get_logger(
      "nav2_costmap_2d"), "GradientLayer::onFootprintChanged(): num footprint points: %lu",
    layered_costmap_->getFootprint().size());
}

// The method is called when costmap recalculation is required.
// It updates the costmap within its window bounds.
// Inside this method the costmap gradient is generated and is writing directly
// to the resulting costmap master_grid without any merging with previous layers.
void YoloLayer::updateCosts(
    nav2_costmap_2d::Costmap2D & master,
    int min_i, int min_j,
    int max_i, int max_j)
{
    std::lock_guard<std::mutex> lock(detection_mutex_);

    // --- ALWAYS PUBLISH MARKERS ---
    m_array_.markers.clear();
    int id = 0;

    auto node = node_.lock();
    if (!node) {
        // always publish empty array to clear old markers
        detection_pub_->publish(m_array_);
        return;
    }

    // Add markers for current detections
    for (const auto &det : detections_) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = global_frame_;
        m.header.stamp = node->get_clock()->now();
        m.ns = "yolo_inflation";
        m.id = id++;
        m.type = visualization_msgs::msg::Marker::CYLINDER;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = det.x;
        m.pose.position.y = det.y;
        m.scale.x = avoidance_radius_ * 2.0;
        m.scale.y = avoidance_radius_ * 2.0;
        m.scale.z = 0.02;
        m.color.r = 1.0; m.color.a = 0.5;
        m.lifetime = rclcpp::Duration::from_seconds(1.0);
        m_array_.markers.push_back(m);
    }

    // Explicitly delete any leftover markers from last call
    for (int old_id = id; old_id < last_marker_count_; ++old_id) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = global_frame_;
        m.header.stamp = node->get_clock()->now();
        m.ns = "yolo_inflation";
        m.id = old_id;
        m.action = visualization_msgs::msg::Marker::DELETE;
        m_array_.markers.push_back(m);
    }

    // Publish all markers once
    detection_pub_->publish(m_array_);

    // Remember how many markers we published (ADD markers only)
    last_marker_count_ = id;

    // --- COSTMAP UPDATE SECTION ---
    if (!need_recalculation_) return;

    // Directly modify master grid instead of private costmap
    const double resolution = master.getResolution();
    const unsigned int size_x = master.getSizeInCellsX();
    const unsigned int size_y = master.getSizeInCellsY();

    // Process all valid detections
    for (const auto& det : persistent_detections_) {
        unsigned int mx, my;
        if (!master.worldToMap(det.x, det.y, mx, my)) continue;

        // Calculate inflation area
        const int cell_radius = static_cast<int>(avoidance_radius_ / resolution);
        const int min_x = std::max(0, static_cast<int>(mx) - cell_radius);
        const int max_x = std::min(static_cast<int>(size_x)-1, static_cast<int>(mx) + cell_radius);
        const int min_y = std::max(0, static_cast<int>(my) - cell_radius);
        const int max_y = std::min(static_cast<int>(size_y)-1, static_cast<int>(my) + cell_radius);

        for (int iy = min_y; iy <= max_y; ++iy) {
            for (int ix = min_x; ix <= max_x; ++ix) {
                // Calculate distance-based cost
                double wx, wy;
                master.mapToWorld(ix, iy, wx, wy);
                const double dist = std::hypot(wx - det.x, wy - det.y);
                
                if (dist > avoidance_radius_) continue;

                unsigned char cost = static_cast<unsigned char>(
                    (1.0 - dist/avoidance_radius_) * 
                    (LETHAL_OBSTACLE - FREE_SPACE) +  // Use LETHAL as base
                    FREE_SPACE
                );

                // Directly update master costmap
                master.setCost(ix, iy, std::max(master.getCost(ix, iy), cost));
            }
        }
    }

}


}  // namespace yolo_costmap_package 

// This is the macro allowing a yolo_costmap_package::YoloLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(yolo_costmap_package::YoloLayer, nav2_costmap_2d::Layer)