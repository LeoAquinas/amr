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
  declareParameter("decay_time", rclcpp::ParameterValue(2.0));


  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "avoidance_radius", avoidance_radius_);
  node->get_parameter(name_ + "." + "subscription_topic", sub_topic_);
  node->get_parameter(name_ + "." + "decay_time", decay_time_); 

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
    auto node = node_.lock();
    if (!node) return;

    // Transform to MAP frame (hardcoded for static frame)
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer_->lookupTransform(
            "map",  // Hardcoded target frame
            msg->header.frame_id,
            tf2::TimePointZero,
            tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(node->get_logger(), "TF error: %s", ex.what());
        return;
    }

    // Process cloud
    sensor_msgs::msg::PointCloud2 transformed_cloud;
    tf2::doTransform(*msg, transformed_cloud, transform);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(transformed_cloud, *cloud);

    // Add new detections with current time
    auto now = node->now();
    for (const auto& point : cloud->points) {
        detections_.emplace_back(point.x, point.y, now);
    }

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
    auto now = node_.lock()->now();

    // Prune old detections FIRST
    detections_.erase(
        std::remove_if(detections_.begin(), detections_.end(),
            [now, this](const Detection& d) {
                return (now - d.stamp).seconds() > decay_time_;
            }),
        detections_.end()
    );

    // Calculate bounds using FRESH detections
    double det_min_x = std::numeric_limits<double>::max();
    double det_min_y = std::numeric_limits<double>::max();
    double det_max_x = -std::numeric_limits<double>::max();
    double det_max_y = -std::numeric_limits<double>::max();

    for (const auto& det : detections_) {
        det_min_x = std::min(det_min_x, det.x - avoidance_radius_);
        det_min_y = std::min(det_min_y, det.y - avoidance_radius_);
        det_max_x = std::max(det_max_x, det.x + avoidance_radius_);
        det_max_y = std::max(det_max_y, det.y + avoidance_radius_);
    }

    // Expand costmap update area
    *min_x = std::min(*min_x, det_min_x);
    *min_y = std::min(*min_y, det_min_y);
    *max_x = std::max(*max_x, det_max_x);
    *max_y = std::max(*max_y, det_max_y);

    // Keep history for next iteration
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
    auto node = node_.lock();
    if (!node) return;

    // Always clear previous markers
    m_array_.markers.clear();
    int marker_id = 0;

    // Get latest map → odom transform
    geometry_msgs::msg::TransformStamped map_to_odom;
    try {
        map_to_odom = tf_buffer_->lookupTransform(
            "odom",  // Target frame (local costmap's frame)
            "map",   // Source frame (where detections are stored)
            tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(node->get_logger(), "TF error: %s", ex.what());
        return;
    }

    // Process detections
    const double resolution = master.getResolution();
    for (const auto& det : detections_) {
        // ========== Visualization (map frame) ==========
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = node->now();
        marker.ns = "yolo_obstacles";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = det.x;
        marker.pose.position.y = det.y;
        marker.scale.x = avoidance_radius_ * 2.0;
        marker.scale.y = avoidance_radius_ * 2.0;
        marker.scale.z = 0.1;
        marker.color.r = 1.0;
        marker.color.a = 0.5;
        marker.lifetime = rclcpp::Duration::from_seconds(0.5);
        m_array_.markers.push_back(marker);

        // ========== Costmap Update (odom frame) ==========
        // Transform detection from map to odom
        tf2::Vector3 det_map(det.x, det.y, 0);
        tf2::Vector3 det_odom = tf2::Transform(tf2::Quaternion(
            map_to_odom.transform.rotation.x,
            map_to_odom.transform.rotation.y,
            map_to_odom.transform.rotation.z,
            map_to_odom.transform.rotation.w
        )) * det_map + tf2::Vector3(
            map_to_odom.transform.translation.x,
            map_to_odom.transform.translation.y,
            map_to_odom.transform.translation.z
        );

        // Inflate around transformed position
        unsigned int mx, my;
        if (!master.worldToMap(det_odom.x(), det_odom.y(), mx, my)) continue;

        const int radius_cells = static_cast<int>(avoidance_radius_ / resolution);
        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
            for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
                const int x = mx + dx;
                const int y = my + dy;
                if (x < 0 || y < 0 || x >= master.getSizeInCellsX() || y >= master.getSizeInCellsY()) 
                    continue;

                // Calculate distance in odom frame
                double wx, wy;
                master.mapToWorld(x, y, wx, wy);
                const double dist = std::hypot(wx - det_odom.x(), wy - det_odom.y());
                
                if (dist > avoidance_radius_) continue;

                unsigned char cost = static_cast<unsigned char>(
                    (1.0 - (dist / avoidance_radius_)) * 
                    (LETHAL_OBSTACLE - FREE_SPACE) + FREE_SPACE
                );
                master.setCost(x, y, std::max(master.getCost(x, y), cost));
            }
        }
    }

    // Delete leftover markers
    for (int id = marker_id; id < last_marker_count_; ++id) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = node->now();
        marker.ns = "yolo_obstacles";
        marker.id = id;
        marker.action = visualization_msgs::msg::Marker::DELETE;
        m_array_.markers.push_back(marker);
    }
    detection_pub_->publish(m_array_);
    last_marker_count_ = marker_id;
}


}  // namespace yolo_costmap_package 

// This is the macro allowing a yolo_costmap_package::YoloLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(yolo_costmap_package::YoloLayer, nav2_costmap_2d::Layer)
