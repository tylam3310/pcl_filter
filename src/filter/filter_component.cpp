#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "pcl_filter/filter/filter_component.hpp"


FilterComponent::FilterComponent() : Node("pclsub")
{
  subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/lidar/points", 
    10, 
    std::bind(&FilterComponent::timer_callback, this, std::placeholders::_1)
  );

  using namespace std::chrono_literals;
  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pcl_data", 10);
}


void FilterComponent::timer_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr pass_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*cloud_msg, *cloud);
  pcl::fromROSMsg(*cloud_msg, *pass_cloud);
  pcl::fromROSMsg(*cloud_msg, *voxel_cloud);

  RCLCPP_INFO(this->get_logger(), "points_size(%d,%d)",cloud_msg->height,cloud_msg->width);

  // define a new container for the data
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);

  // PassThrough Filter
  pcl::PassThrough<pcl::PointXYZI> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");  // x axis
  // extract point cloud between 1.0 and 3.0 m
  pass.setFilterLimits(0.0, 1.9);
  // pass.setFilterLimitsNegative (true);   // extract range reverse
  pass.filter(*pass_cloud);

  // Approximate Voxel Grid
  pcl::ApproximateVoxelGrid<pcl::PointXYZI> avg;
  avg.setInputCloud(pass_cloud);
  avg.setLeafSize(0.1f, 0.1f, 0.1f);
  avg.setDownsampleAllData(true);
  avg.filter(*voxel_cloud);

  // Radius Outlier Removal
  pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem;
  outrem.setInputCloud(voxel_cloud);
  outrem.setRadiusSearch(0.6);
  outrem.setMinNeighborsInRadius(2);
  outrem.setKeepOrganized(true);
  outrem.filter(*cloud_filtered);

  // Statistical Outlier Removal
  // pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
  // sor.setInputCloud(cloud);
  // sor.setMeanK(10);
  // sor.setStddevMulThresh(0.2);
  // sor.setNegative(true);
  // sor.filter (*cloud_filtered);

  // Conditional Removal
  // pcl::ConditionAnd<pcl::PointXYZI>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZI>());
  // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("z", pcl::ComparisonOps::GT, 0.0)));
  // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("z", pcl::ComparisonOps::LT, 1.7)));
  // pcl::ConditionalRemoval<pcl::PointXYZI> condrem;
  // condrem.setCondition(range_cond);
  // condrem.setInputCloud(cloud);
  // // condrem.setKeepOrganized(true);
  // condrem.filter(*cloud_filtered);
  // vector<int> Idx;
  // pcl::removeNaNFromPointCloud(*cloud_filtered, *cloud_filtered, Idx);


  sensor_msgs::msg::PointCloud2 sensor_msg;
  pcl::toROSMsg(*cloud_filtered, sensor_msg);
  publisher_->publish(sensor_msg);
}