#include <chrono>
#include <iostream>
#include <sys/socket.h>
#include <sys/types.h>
#include <cstring>
#include <arpa/inet.h>
#include <netinet/in.h>
#include "rclcpp/rclcpp.hpp"
#include "arpis_network/tcp/tcp.hpp"

#include <math.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#define _USE_MATH_DEFINES

using namespace std::chrono_literals;

class ArpisClientNode : public rclcpp::Node {
  public:
  ArpisClientNode(): Node("client") {
    tcp = new arpis_network::tcp("127.0.0.1", 3030);
    tcp->connect();
    tcp_receiver = this->create_publisher<geometry_msgs::msg::Quaternion>("tcp_receiver", 5);
    timer_ = this->create_wall_timer(400ms, std::bind(&ArpisClientNode::exec, this));
    // this->exec();
  }
  private:  
  void rviz() {

  }
  void exec() {    
    char buffer[1024] = {0};
    tcp->receive(buffer, 1024);
    if (std::string(buffer).length() != 0)
      std::cout << std::string(buffer) << std::endl;    
  }  
  rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr tcp_receiver;
  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr rcp_subscription;
  rclcpp::TimerBase::SharedPtr timer_;
  arpis_network::tcp * tcp;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // auto node = std::make_shared<rclcpp::Node>()
  rclcpp::spin(std::make_shared<ArpisClientNode>());
  rclcpp::shutdown();
  
  return 0;
}
