#include "square_service/client.hpp"
#define CLIENT_NAME "square_root_integer_client"

/*
input: none.
output: Initializes the SquareIntClient node and creates the client for the square_int service.
description: Constructs a SquareIntClient object by initializing the ROS2 node with the client name and setting up the client to send requests to the square_int service.
*/
SquareIntClient::SquareIntClient(): Node(CLIENT_NAME)
{
    client_ = create_client<square_service::srv::SquareInt>("square_int");
}


/*
input:
    - num: The integer value to be sent as a request to the square_int service.
output: none.
description: Waits for the square_int service to be available, then creates and sends an asynchronous request containing the provided number. Once the response is received, it logs the details of the square, cube, square root, and whether the square root is perfect.
*/
void SquareIntClient::send_request(int64_t num)
{
    //waiting for service
    while(!client_->wait_for_service(std::chrono::seconds(1)))
    {
        if(!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for service...");
    }
    
    auto request = std::make_shared<square_service::srv::SquareInt::Request>();
    request->number = num;
    
    // Use callback version for async request
    client_->async_send_request
    (
        request,
        [this, num](rclcpp::Client<square_service::srv::SquareInt>::SharedFuture future) 
        {
            std::string isPrefectSquareRoot = future.get()->is_perfect_square_root ? "true": "false";
            RCLCPP_INFO(this->get_logger(), "Number: %ld", num);
            RCLCPP_INFO(this->get_logger(), "Send Response - Squred Number: %ld & Cubed Number: %ld & Square Root Number: %lf & is perfect squareRoot: %s", future.get()->square, future.get()->cube, future.get()->square_root, isPrefectSquareRoot.c_str());
        }
    );
}