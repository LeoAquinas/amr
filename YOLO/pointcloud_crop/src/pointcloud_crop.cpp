#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <yolo_msgs/msg/inference_array.hpp>

class CropBoxNode : public rclcpp::Node {
public:
  CropBoxNode() : Node("crop_box_node") {
    pc_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/depth_camera/points", 1,
      std::bind(&CropBoxNode::cloudCb, this, std::placeholders::_1));

    yolo_sub_ = create_subscription<yolo_msgs::msg::InferenceArray>(
      "/camera/inference_data", 1,
      std::bind(&CropBoxNode::dataCallback, this, std::placeholders::_1));

    cropped_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/camera/cropped_cloud", 10);
  }

private:
  void cloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    latest_cloud_ = msg;
    processPointCloud();
  }

  void dataCallback(const yolo_msgs::msg::InferenceArray::SharedPtr msg) {
    latest_yolo_results_ = msg;
    processPointCloud();
  }

  void processPointCloud() {
    if (!latest_cloud_ || !latest_yolo_results_) {
      RCLCPP_INFO(this->get_logger(), "Waiting for both point cloud and detection data...");
      return;
    }

    // Convert ROS2 to PCL
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*latest_cloud_, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

    // Create combined cloud for all detections
    pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    const int width_px = 128;  // Update with actual camera resolution
    const int height_px = 96;  // Update with actual camera resolution

    for (const auto& detection : latest_yolo_results_->inference_result) {
      // Convert YOLO pixel coordinates to normalized device coordinates
      float xmin_ndc = 3.0f * (detection.left / static_cast<float>(width_px)) - 1.5f;
      float xmax_ndc = 3.0f * (detection.right / static_cast<float>(width_px)) - 1.5f;
      float ymin_ndc = 3.0f * (detection.bottom / static_cast<float>(height_px)) - 1.5f;
      float ymax_ndc = 3.0f * (detection.top / static_cast<float>(height_px)) - 1.5f;

      RCLCPP_INFO(this->get_logger(), "xmin_ndc, %ld", detection.left);
      RCLCPP_INFO(this->get_logger(), "xmax_ndc, %ld", detection.right);
      RCLCPP_INFO(this->get_logger(), "ymin_ndc, %ld", detection.bottom);
      RCLCPP_INFO(this->get_logger(), "ymax_ndc, %ld", detection.top);

      RCLCPP_INFO(this->get_logger(), "xmin_ndc, %f", xmin_ndc);
      RCLCPP_INFO(this->get_logger(), "xmax_ndc, %f", xmax_ndc);
      // Configure crop box for this detection
      pcl::CropBox<pcl::PointXYZ> crop;
      crop.setInputCloud(cloud);

      // Safety buffer for width
      float safe_xmin_ndc = xmin_ndc - 0.2;
      float safe_xmax_ndc = xmax_ndc + 0.2;
      
      Eigen::Vector4f min_pt(
        safe_xmin_ndc, 
        -1.0f,
        1.0f,  // Minimum Z (1 meter)
        1.0f
      );
      
      Eigen::Vector4f max_pt(
        safe_xmax_ndc,
        1.0f,
        3.0f,  // Maximum Z (3 meters)
        1.0f
      );

      crop.setMin(min_pt);
      crop.setMax(max_pt);

      // Apply filter
      pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      crop.filter(*cropped_cloud);

      // Add to combined cloud
      *combined_cloud += *cropped_cloud;
    }

    // Convert back to ROS2 and publish
    // Always publish, even if it's empty
    pcl::PCLPointCloud2 combined_pc2;
    pcl::toPCLPointCloud2(*combined_cloud, combined_pc2);

    // Convert to ROS2 message
    sensor_msgs::msg::PointCloud2 out;
    pcl_conversions::fromPCL(combined_pc2, out);

    // Copy the original header (frame, timestamp)
    out.header = latest_cloud_->header;

    // If there are no points, force width/height = 0 and clear the data
    if (combined_cloud->empty()) {
      out.width  = 0;
      out.height = 0;
      out.row_step   = 0;
      out.data.clear();
      // You can still leave fields & point_step so that subscribers know the layout
    }

    // Publish your (possibly empty) cloud
    cropped_pub_->publish(out);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
  rclcpp::Subscription<yolo_msgs::msg::InferenceArray>::SharedPtr yolo_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cropped_pub_;

  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
  yolo_msgs::msg::InferenceArray::SharedPtr latest_yolo_results_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CropBoxNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}