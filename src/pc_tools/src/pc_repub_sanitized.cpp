#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
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

    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel.filter(*filtered);

    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(*filtered, out);

    out.header = msg->header;
    out.is_dense = false;

    pub_->publish(out);

    // RCLCPP_INFO_THROTTLE(
    //   this->get_logger(),
    //   *this->get_clock(),
    //   1000,
    //   "published filtered cloud: in=%zu out=%zu leaf=%.3f",
    //   cloud->size(),
    //   filtered->size(),
    //   leaf_size
    // );
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
