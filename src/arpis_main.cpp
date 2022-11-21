#include <chrono>
#include <iostream>
#include <sys/socket.h>
#include <sys/types.h>
#include <cstring>
#include <arpa/inet.h>
#include <netinet/in.h>
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class ArpisClientNode : public rclcpp::Node {
  public:
  ArpisClientNode(): Node("client") {
    // timer_ = this->create_wall_timer(400ms, std::bind(&ArpisClientNode::exec, this));
    this->exec();
  }
  private:
  void exec() {
    sockaddr_in serv;
    bzero(&serv, sizeof(serv));
    char buffer[1024];
    int sock;
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
      std::cout << "Faillied created socket" << std::endl;
    } else 
      std::cout << "success created socket" << std::endl;

    // memset(&serv, '\0', sizeof(serv));
    serv.sin_port = htons(3030);
    serv.sin_family = AF_INET;
    serv.sin_addr.s_addr = inet_addr("127.0.0.1");        
    if ( connect(sock, (sockaddr*)&serv, sizeof(serv)) != 0 ) {
      std::cout << "failied sconnect server" << std::endl;
    } else 
      std::cout << "success connect server" << std::endl;

  }
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArpisClientNode>());
  rclcpp::shutdown();
  
  return 0;
}
