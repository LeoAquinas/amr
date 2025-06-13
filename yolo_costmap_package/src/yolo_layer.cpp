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
  declareParameter("inscribed_radius",  rclcpp::ParameterValue(0.05));
  declareParameter("avoidance_radius",  rclcpp::ParameterValue(0.5));
  declareParameter("subscription_topic", rclcpp::ParameterValue(std::string("")));
  declareParameter("decay_time", rclcpp::ParameterValue(2.0));


  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "inscribed_radius",  inscribed_radius_);
  node->get_parameter(name_ + "." + "avoidance_radius", avoidance_radius_);
  node->get_parameter(name_ + "." + "subscription_topic", sub_topic_);
  node->get_parameter(name_ + "." + "decay_time", decay_time_); 

  global_frame_ = layered_costmap_->getGlobalFrameID();
  resolution_ = layered_costmap_->getCostmap()->getResolution();

  pointcloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    sub_topic_, rclcpp::SensorDataQoS(),  // Use sensor QoS
    std::bind(&YoloLayer::pointcloudCallback, this, std::placeholders::_1));

  detection_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "detection/detected_cloud", 10);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize costmap and cache
  costmap_ = layered_costmap_->getCostmap();
  computeCaches();

  clock_ = node->get_clock();
  last_callback_time_ = clock_->now();

  last_marker_count_ = 0;

  need_recalculation_ = false;
  current_ = true;
}

void YoloLayer::computeCaches() {
  if (avoidance_radius_ <= 0) return;

  cell_inflation_radius_ = static_cast<int>(std::ceil(avoidance_radius_ / resolution_));
  cache_length_ = cell_inflation_radius_ + 2;

  // Precompute cost matrix using Euclidean distance
  cached_cost_matrix_.resize(cache_length_, std::vector<unsigned char>(cache_length_, FREE_SPACE));
  for (int j = 0; j < cache_length_; ++j) {
    for (int i = 0; i < cache_length_; ++i) {
      const double distance = std::hypot(i, j) * resolution_;
      if (distance > avoidance_radius_) 
        continue;
      
      if (distance <= inscribed_radius_) {
        cached_cost_matrix_[i][j] = INSCRIBED_INFLATED_OBSTACLE;
      } else {
        const double factor = (distance - inscribed_radius_) / (avoidance_radius_ - inscribed_radius_);
        cached_cost_matrix_[i][j] = static_cast<unsigned char>(
          (1.0 - factor) * (INSCRIBED_INFLATED_OBSTACLE - FREE_SPACE) + FREE_SPACE);
      }
    }
  }
}

void YoloLayer::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    const rclcpp::Time now = clock_->now();
    if ((now - last_callback_time_).seconds() < 0.1) {  // 10 Hz max
        return;
    }
    last_callback_time_ = now;

    auto node = node_.lock();
    if (!node) return;

    // Get current transform to avoid extrapolation errors
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer_->lookupTransform(
            global_frame_,
            msg->header.frame_id,
            msg->header.stamp,
            tf2::durationFromSec(5.0));
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(node->get_logger(), "TF error: %s", ex.what());
        return;
    }

    // Process cloud
    sensor_msgs::msg::PointCloud2 transformed_cloud;
    tf2::doTransform(*msg, transformed_cloud, transform);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(transformed_cloud, *cloud);

    // Add new detections
    std::lock_guard<std::mutex> lock(detection_mutex_);
    
    // Remove expired detections FIRST
    detections_.erase(
        std::remove_if(detections_.begin(), detections_.end(),
            [now, this](const Detection& d) {
                return (now - d.stamp).seconds() > decay_time_;
            }),
        detections_.end()
    );

    bool did_change = false;
    merge_threshold_ = 0.05;
    // 2) Merge new detections (only mark change=true if a fresh point is farther than merge_threshold)
    for (const auto& point : cloud->points) {
        bool found = false;
        for (auto& det : detections_) {
            if (std::hypot(point.x - det.x, point.y - det.y) < merge_threshold_) {
                det.stamp = now;  // refresh timestamp
                found = true;
                break;
            }
        }
        if (!found) {
            detections_.emplace_back(point.x, point.y, now);
            did_change = true;
        }
    }
    RCLCPP_INFO(node->get_logger(),
    "[YoloLayer] raw cloud pts=%zu, total detections after merge=%zu",
    cloud->points.size(), detections_.size());
    // 3) If any detection expired in (1), or we added a new one in (2), THEN recalc
    if (did_change || detections_.size() < last_detection_count_) {
        need_recalculation_ = true;
    }
    last_detection_count_ = detections_.size();
}


// The method is called to ask the plugin: which area of costmap it needs to update.
// Inside this method window bounds are re-calculated if need_recalculation_ is true
// and updated independently on its value.
void YoloLayer::updateBounds(
    double robot_x, double robot_y, double /*robot_yaw*/,
    double* min_x, double* min_y, double* max_x, double* max_y)
{
    if (!need_recalculation_) {
        return;
    }

    std::lock_guard<std::mutex> lock(detection_mutex_);
    const auto now = clock_->now();

    // Prune old detections FIRST
    detections_.erase(
        std::remove_if(detections_.begin(), detections_.end(),
            [now, this](const Detection& d) {
                return (now - d.stamp).seconds() > decay_time_;
            }),
        detections_.end()
    );

    if (detections_.empty()) {
        // Nothing to add → no need to expand beyond last bounds
        return;
    }


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
    // Always process if there are detections OR we need to clear expired ones
    if (!need_recalculation_ && detections_.empty()) {
        return;
    }

    std::vector<Detection> current_detections;
    {
        std::lock_guard<std::mutex> lock(detection_mutex_);
        current_detections = detections_;
    }

    // Clear previous detection areas FIRST
    unsigned char* master_array = master.getCharMap();
    // 1) First, clear every cell we inflated last time
    for (auto & xy : last_inflated_cells_) {
    unsigned int mx = xy.first, my = xy.second;
    master_array[ master.getIndex(mx, my) ] = FREE_SPACE;
    }
    last_inflated_cells_.clear();

    // 2) Now re‐inflate current detections
    for (const auto& det : current_detections) {
        unsigned int mx, my;
        if (!master.worldToMap(det.x, det.y, mx, my)) {
            continue;
        }
        // Mark the center cell as lethal
        master_array[ master.getIndex(mx,my) ] = LETHAL_OBSTACLE;
        last_inflated_cells_.push_back({mx,my});

        // Inflate outwards
        for (int dx = -cell_inflation_radius_; dx <= cell_inflation_radius_; dx++) {
            for (int dy = -cell_inflation_radius_; dy <= cell_inflation_radius_; dy++) {
            unsigned int nx = mx + dx, ny = my + dy;
            if (nx < (unsigned)min_i || ny < (unsigned)min_j ||
                nx >= (unsigned)max_i || ny >= (unsigned)max_j) {
                continue;
            }
            double dist = std::hypot(dx,dy) * resolution_;
            if (dist > avoidance_radius_) {
                continue;
            }
            unsigned char new_cost = computeCost(dist);
            unsigned int index = master.getIndex(nx, ny);
            if (new_cost > master_array[index]) {
                master_array[index] = new_cost;
            }
            last_inflated_cells_.push_back({nx,ny});
            }
        }
    }
    need_recalculation_ = false;
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;

    for (int j = min_j; j < max_j; j++) {
        for (int i = min_i; i < max_i; i++) {
            unsigned char our_cost = master.getCost(i, j);
            if (our_cost > FREE_SPACE) {
                double wx, wy;
                master.mapToWorld(i, j, wx, wy);

                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = global_frame_;
                marker.header.stamp = clock_->now();
                marker.ns = "inflated_cells";
                marker.id = id++;
                marker.type = visualization_msgs::msg::Marker::CUBE;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.pose.position.x = wx;
                marker.pose.position.y = wy;
                marker.pose.position.z = 0.05;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = resolution_;
                marker.scale.y = resolution_;
                marker.scale.z = 0.01;
                marker.color.a = 0.6;
                marker.color.r = 1.0;
                marker.color.g = 1.0 - our_cost / 255.0;
                marker.color.b = 0.0;
                marker.lifetime = rclcpp::Duration::from_seconds(0.2);

                marker_array.markers.push_back(marker);
            }
        }
    }

    detection_pub_->publish(marker_array);

}

void YoloLayer::enqueue(
    int x, int y,
    unsigned int src_x, unsigned int src_y,
    unsigned int size_x)
{
    // Cast to int for abs calculation
    const int dx = std::abs(x - static_cast<int>(src_x));
    const int dy = std::abs(y - static_cast<int>(src_y));
    const int distance = std::max(dx, dy);  // Chebyshev distance for binning
    
    if (distance > cell_inflation_radius_) {
        return;
    }

    const unsigned int index = y * size_x + x;
    if (index >= seen_.size() || seen_[index]) return;
    
    inflation_cells_[distance].emplace_back(x, y, src_x, src_y);
}

unsigned char YoloLayer::computeCost(double distance) const {
    if (distance <= inscribed_radius_) {
        return LETHAL_OBSTACLE;
    } else if (distance > avoidance_radius_) {
        return FREE_SPACE;
    }
    
    const double factor = (distance - inscribed_radius_) / 
                         (avoidance_radius_ - inscribed_radius_);
    return static_cast<unsigned char>(
        (1.0 - factor) * (INSCRIBED_INFLATED_OBSTACLE - FREE_SPACE) + FREE_SPACE
    );
}




}  // namespace yolo_costmap_package 

// This is the macro allowing a yolo_costmap_package::YoloLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(yolo_costmap_package::YoloLayer, nav2_costmap_2d::Layer)