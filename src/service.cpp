#include "square_service/service.hpp"
#include "square_service/srv/square_int.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

#define SERVICE_NAME "square_root_integer_service"

SquareIntService::SquareIntService():Node(SERVICE_NAME)
{
    service_ = create_service<square_service::srv::SquareInt>(
        "square_int",
        std::bind(&SquareIntService::handle_service, this, 
            std::placeholders::_1, std::placeholders::_2));
}


void SquareIntService::handle_service(const std::shared_ptr<square_service::srv::SquareInt::Request> request, std::shared_ptr<square_service::srv::SquareInt::Response> response)
{
    int64_t input_num = request->number;
    response->square = pow(input_num, 0.5);
    response->cube = pow(input_num, 2);
    RCLCPP_INFO(this->get_logger(), "Got request - Number %ld", request->number);
    RCLCPP_INFO(this->get_logger(), "Send Response - Squred Number: %lf & Cubed Number: %ld", response->square, response->cube);
}