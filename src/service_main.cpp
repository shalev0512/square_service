#include "iostream"
#include "square_service/service.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto serviceNode = std::make_shared<SquareIntService>();
    rclcpp::spin(serviceNode);
    rclcpp::shutdown();
    return 0;
}