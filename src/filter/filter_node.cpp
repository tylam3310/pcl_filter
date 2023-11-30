#include "rclcpp/rclcpp.hpp"
#include "pcl_filter/filter/filter_component.hpp"

int main(int argc, char **argv)
{   
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FilterComponent>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}