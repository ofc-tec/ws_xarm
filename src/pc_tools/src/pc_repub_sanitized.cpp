#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

class PCRepubSanitized : public rclcpp::Node
{
public:
  PCRepubSanitized() : Node("pc_repub_sanitized_cpp")
  {
    this->declare_parameter<double>("leaf_size", 0.02);
    this->declare_parameter<int>("keep_every", 2);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/camera/depth/points_repub", qos);

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/depth/points",
      qos,
      std::bind(&PCRepubSanitized::cb, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "C++ pointcloud sanitizer running");
  }

private:
  void cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    count_++;

    int keep_every = this->get_parameter("keep_every").as_int();
    if (keep_every > 1 && (count_ % keep_every) != 0) {
      return;
    }

    double leaf_size = this->get_parameter("leaf_size").as_double();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::fromROSMsg(*msg, *cloud);

    size_t finite_count = 0;
    float min_x = std::numeric_limits<float>::infinity();
    float min_y = std::numeric_limits<float>::infinity();
    float min_z = std::numeric_limits<float>::infinity();
    float max_x = -std::numeric_limits<float>::infinity();
    float max_y = -std::numeric_limits<float>::infinity();
    float max_z = -std::numeric_limits<float>::infinity();
    for (const auto & p : cloud->points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }
      finite_count++;
      min_x = std::min(min_x, p.x);
      min_y = std::min(min_y, p.y);
      min_z = std::min(min_z, p.z);
      max_x = std::max(max_x, p.x);
      max_y = std::max(max_y, p.y);
      max_z = std::max(max_z, p.z);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr clean(new pcl::PointCloud<pcl::PointXYZ>());
    std::vector<int> valid_indices;
    cloud->is_dense = false;
    pcl::removeNaNFromPointCloud(*cloud, *clean, valid_indices);

    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(clean);
    voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel.filter(*filtered);

    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(*filtered, out);

    out.header = msg->header;
    out.is_dense = false;

    pub_->publish(out);

    std::ostringstream fields;
    for (size_t i = 0; i < msg->fields.size(); ++i) {
      if (i > 0) {
        fields << ",";
      }
      fields << msg->fields[i].name << "@" << msg->fields[i].offset;
    }

    // RCLCPP_INFO_THROTTLE(
    //   this->get_logger(),
    //   *this->get_clock(),
    //   1000,
    //   "pc debug frame=%s raw=%ux%u fields=[%s] point_step=%u row_step=%u "
    //   "pcl=%zu finite=%zu clean=%zu filtered=%zu out=%ux%u leaf=%.3f "
    //   "xyz_min=(%.3f,%.3f,%.3f) xyz_max=(%.3f,%.3f,%.3f)",
    //   msg->header.frame_id.c_str(),
    //   msg->width,
    //   msg->height,
    //   fields.str().c_str(),
    //   msg->point_step,
    //   msg->row_step,
    //   cloud->size(),
    //   finite_count,
    //   clean->size(),
    //   filtered->size(),
    //   out.width,
    //   out.height,
    //   leaf_size,
    //   finite_count ? min_x : 0.0f,
    //   finite_count ? min_y : 0.0f,
    //   finite_count ? min_z : 0.0f,
    //   finite_count ? max_x : 0.0f,
    //   finite_count ? max_y : 0.0f,
    //   finite_count ? max_z : 0.0f);
  }

  int count_ = 0;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PCRepubSanitized>());
  rclcpp::shutdown();
  return 0;
}
