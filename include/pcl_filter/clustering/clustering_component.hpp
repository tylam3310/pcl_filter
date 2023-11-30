#ifndef PCL_FILTER__CLUSTERING_COMPONENT_HPP_
#define PCL_FILTER__CLUSTERING_COMPONENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class ClusteringComponent : public rclcpp::Node
{
  public:
    ClusteringComponent();

  private:
    void timer_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
    size_t count_;
};

#endif // PCL_FILTER__CLUSTERING_COMPONENT_HPP_