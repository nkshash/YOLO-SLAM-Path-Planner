/// \file
/// \brief This node removes and downsamples noisy point cloud data
///

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

class PointCloudProcessing : public rclcpp::Node
{
public:
  PointCloudProcessing() : Node("point_cloud_processing")
  {
    // Initialize parameters
    declare_parameters();

    // Set up the publisher, subscriber, and timer
    setup_ros_components();
  }

private:
  // Function to declare parameters with default values
  void declare_parameters()
  {
    declare_parameter("x_filter_min", -0.5);
    declare_parameter("x_filter_max", 0.0);
    declare_parameter("z_filter_min", -0.26);
    declare_parameter("z_filter_max", 0.13);
    declare_parameter("search_radius", 0.8);
    declare_parameter("num_neighbors", 2);
    declare_parameter("voxel_leaf_size", 0.01);
  }

  // Set up ROS components like publishers, subscribers, and timers
  void setup_ros_components()
  {
    get_parameters();

    // Set the publishing rate
    rate_ = 200;

    // Create publisher for filtered point cloud
    filtered_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_velodyne_points", 10);

    // Create subscriber for incoming point cloud
    velodyne_points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/velodyne_points", 10, std::bind(&PointCloudProcessing::velodyne_points_callback, this, std::placeholders::_1));

    // Create a timer to publish filtered data at the specified rate
    timer_ = create_wall_timer(
        std::chrono::milliseconds(1000 / rate_),
        std::bind(&PointCloudProcessing::timer_callback, this));
  }

  // Retrieve parameters' values
  void get_parameters()
  {
    x_filter_min_ = declare_parameter("x_filter_min", -0.5);
    x_filter_max_ = declare_parameter("x_filter_max", 0.0);
    z_filter_min_ = declare_parameter("z_filter_min", -0.26);
    z_filter_max_ = declare_parameter("z_filter_max", 0.13);
    search_radius_ = declare_parameter("search_radius", 0.8);
    num_neighbors_ = declare_parameter("num_neighbors", 2);
    voxel_leaf_size_ = declare_parameter("voxel_leaf_size", 0.01);
  }

  // Callback function for processing incoming point cloud data
  void velodyne_points_callback(const sensor_msgs::msg::PointCloud2 &msg)
  {
    // Convert ROS2 PointCloud2 to PointCloud<PointXYZI>
    pcl::fromROSMsg(msg, *pcl_cloud_);

    // Filter the point cloud data
    pcl_cloud_ = filter(pcl_cloud_);

    // Convert PointCloud<PointXYZI> to ROS2 PointCloud2
    pcl::toROSMsg(*pcl_cloud_, cloud_);
  }

  // Function to perform filtering operations on point cloud data
  pcl::PointCloud<pcl::PointXYZI>::Ptr filter(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out{new pcl::PointCloud<pcl::PointXYZI>()};
    pcl::PassThrough<pcl::PointXYZI> pt1, pt2;
    pcl::RadiusOutlierRemoval<pcl::PointXYZI> rad_outlier;
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;

    // Filter the data along z-axis
    pt1.setInputCloud(cloud_in);
    pt1.setFilterFieldName("z");
    pt1.setFilterLimits(z_filter_min_, z_filter_max_);
    pt1.filter(*cloud_out);

    // Filter the data along x-axis
    pt2.setInputCloud(cloud_out);
    pt2.setFilterFieldName("x");
    pt2.setFilterLimits(x_filter_min_, x_filter_max_);
    pt2.setNegative(true);
    pt2.filter(*cloud_out);

    // Remove outliers
    rad_outlier.setInputCloud(cloud_out);
    rad_outlier.setRadiusSearch(search_radius_);
    rad_outlier.setMinNeighborsInRadius(num_neighbors_);
    rad_outlier.filter(*cloud_out);

    // Downsample the data
    voxel_grid.setInputCloud(cloud_out);
    voxel_grid.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    voxel_grid.filter(*cloud_out);

    return cloud_out;
  }

  // Callback function to publish the filtered point cloud data
  void timer_callback()
  {
    filtered_points_pub_->publish(cloud_);
  }

  // ROS publishers, subscribers, and variables for parameters and point cloud data
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_points_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr velodyne_points_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters and variables for filtering and processing
  double x_filter_min_, x_filter_max_, z_filter_min_, z_filter_max_, search_radius_;
  int num_neighbors_, rate_;
  float voxel_leaf_size_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_{new pcl::PointCloud<pcl::PointXYZI>()};
  sensor_msgs::msg::PointCloud2 cloud_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudProcessing>());
  rclcpp::shutdown();
  return 0;
}
