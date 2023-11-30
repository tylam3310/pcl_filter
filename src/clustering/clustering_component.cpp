#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include "pcl_filter/clustering/clustering_component.hpp"


ClusteringComponent::ClusteringComponent() : Node("pclsub")
{
  subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/lidar/points", //topics of pointcloud2
    1, 
    std::bind(&ClusteringComponent::timer_callback, this, std::placeholders::_1)\
  );

  using namespace std::chrono_literals;
  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pcl_data", 1);
}

void ClusteringComponent::timer_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*cloud_msg, *cloud);

  RCLCPP_INFO(this->get_logger(), "points_size(%d,%d)",cloud_msg->height,cloud_msg->width);

  // define a new container for the data
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);

  // Voxel Grid: pattern 1
  pcl::VoxelGrid<pcl::PointXYZI> voxelGrid;
  voxelGrid.setInputCloud(cloud);
  // set the leaf size (x, y, z)
  voxelGrid.setLeafSize(0.1, 0.1, 0.1);
  // apply the filter to dereferenced cloudVoxel
  voxelGrid.filter(*cloud_filtered);

  // [pcl::VoxelGrid::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow.

  // SAC Segmentation
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);  
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);  
  // Create the segmentation object  
  pcl::SACSegmentation<pcl::PointXYZI> seg;  
  double threshould = 0.01;
  // Optional  
  seg.setOptimizeCoefficients (true);  
  // Mandatory  
  seg.setModelType (pcl::SACMODEL_PLANE);  
  seg.setMethodType (pcl::SAC_RANSAC);  
  seg.setDistanceThreshold (threshould);  
  seg.setInputCloud (cloud_filtered);  
  seg.segment (*inliers, *coefficients);  

  for (size_t i = 0; i < inliers->indices.size (); ++i) {
    cloud_filtered->points[inliers->indices[i]].x;  
    cloud_filtered->points[inliers->indices[i]].y;  
    cloud_filtered->points[inliers->indices[i]].z;  
  }  

  // Extract the planar inliers from the input cloud
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud (cloud_filtered);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter (*cloud_filtered);


  // Create the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud (cloud_filtered);

  // create the extraction object for the clusters
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ece;
  // specify euclidean cluster parameters
  ece.setClusterTolerance (0.02); // 2cm
  ece.setMinClusterSize (20);
  ece.setMaxClusterSize (10000);
  ece.setSearchMethod (tree);
  ece.setInputCloud (cloud_filtered);
  // exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
  ece.extract (cluster_indices);


  pcl::PCDWriter writer;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);  
  pcl::copyPointCloud(*cloud_filtered, *cloud_cluster);  
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)  
    {  
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++) {  
  cloud_cluster->points[*pit].x;  
  cloud_cluster->points[*pit].y;  
  cloud_cluster->points[*pit].z;  
      }  
      // std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;  
      // std::stringstream ss;  
      // ss << "cloud_cluster_" << j << ".pcd";  
      // writer.write<pcl::PointXYZI> (ss.str (), *cloud_cluster, false);   
    }  

  sensor_msgs::msg::PointCloud2 sensor_msg;
  pcl::toROSMsg(*cloud_cluster, sensor_msg);
  publisher_->publish(sensor_msg);
}
