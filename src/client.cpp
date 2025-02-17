#include "square_service/client.hpp"
#define CLIENT_NAME "square_root_integer_client"

SquareIntClient::SquareIntClient(): Node(CLIENT_NAME)
{
    client_ = create_client<square_service::srv::SquareInt>("square_int");
}

void SquareIntClient::send_request(int64_t num){

    //waiting for service
    while(!client_->wait_for_service(std::chrono::seconds(1))){
        if(!rclcpp::ok()){
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for service...");
    }

    auto request = std::make_shared<square_service::srv::SquareInt::Request>();
    request->number = num;

    // Use callback version for async request
    client_->async_send_request(
        request,
        [this, num](rclcpp::Client<square_service::srv::SquareInt>::SharedFuture future) {
            RCLCPP_INFO(this->get_logger(), "Number: %ld", num);
            RCLCPP_INFO(this->get_logger(), "Square: %lf", future.get()->square);
        }
    );
}