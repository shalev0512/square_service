#ifndef CLIENT_HPP
#define CLIENT_HPP

#include "rclcpp/rclcpp.hpp"
#include "square_service/srv/square_int.hpp"

class SquareIntClient : public rclcpp::Node
{
private:
    rclcpp::Client<square_service::srv::SquareInt>::SharedPtr client_;

public:
    SquareIntClient();
    void send_request(int64_t num);

};

#endif