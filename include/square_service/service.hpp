#ifndef SERVICE_HPP
#define SERVICE_HPP

#include "rclcpp/rclcpp.hpp"
#include "square_service/srv/square_int.hpp"
#include <memory>

class SquareIntService: public rclcpp::Node{

public:
    SquareIntService();
private:
    rclcpp::Service<square_service::srv::SquareInt>::SharedPtr service_;
    void handle_service(const std::shared_ptr<square_service::srv::SquareInt::Request> request, std::shared_ptr<square_service::srv::SquareInt::Response> response);
};

#endif