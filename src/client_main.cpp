#include <iostream>
#include <random>
#include <chrono>
#include <cstdlib>
#include "square_service/client.hpp"

using namespace std::chrono_literals;

#define MIN_RANGE 1
#define MAX_RANGE 101

int main(int argc, char **argv){
    //vars
    int64_t rand_num = 0;

    //Initializing the ros2 cpp libary
    rclcpp::init(argc, argv);
    auto clientNode = std::make_shared<SquareIntClient>();

    rand_num = MIN_RANGE + (std::rand() % (MAX_RANGE-MIN_RANGE+1));
    clientNode->send_request(rand_num);

    auto timer = clientNode->create_wall_timer(1s, [clientNode](){

        //generating a random num between 1-100
        int64_t rand_num = MIN_RANGE + (std::rand() % (MAX_RANGE-MIN_RANGE+1));

        //sending the request
        clientNode->send_request(rand_num);  // Also note: fixed typo in 'request'
    });

    // Add this line to keep the node running
    rclcpp::spin(clientNode);

    rclcpp::shutdown();
    return 0;
}