#include "rclcpp/rclcpp.hpp"
#include "pcl_filter/clustering/clustering_component.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ClusteringComponent>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}