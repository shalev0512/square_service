#include "square_service/service.hpp"
#include "square_service/srv/square_int.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

#define SERVICE_NAME "square_root_integer_service"

SquareIntService::SquareIntService():Node(SERVICE_NAME)
{
    service_ = create_service<square_service::srv::SquareInt>
    (
        "square_int",
        std::bind(&SquareIntService::handle_service, this, std::placeholders::_1, std::placeholders::_2)
    );
}


void SquareIntService::handle_service(const std::shared_ptr<square_service::srv::SquareInt::Request> request, std::shared_ptr<square_service::srv::SquareInt::Response> response)
{
    //vars
    float reminder = 0;
    int64_t input_num = request->number;
    std::string isPrefectSquareRoot = "";

    response->square = static_cast<int64_t>(pow(input_num, 2));
    response->cube = static_cast<int64_t>(pow(input_num, 3));
    response->square_root = static_cast<double>(pow(input_num, 0.5));

    reminder = std::fmod(response->square_root, 1.0);
    response->is_perfect_square_root = (reminder == 0.0);

    isPrefectSquareRoot = response->is_perfect_square_root ? "true": "false";

    RCLCPP_INFO(this->get_logger(), "Got request - Number %ld", request->number);
    RCLCPP_INFO(this->get_logger(), "Send Response - Squred Number: %ld & Cubed Number: %ld & Square Root Number: %lf & is perfect squareRoot: %s", response->square, response->cube, response->square_root, isPrefectSquareRoot.c_str());
}