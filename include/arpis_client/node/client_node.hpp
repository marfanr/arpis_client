#ifndef CLIENT_NODE_HPP_
#define CLIENT_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
// #include "arpis_ne"

namespace arpis_client {

class ClientNode {
public:
    ClientNode(rclcpp::Node::SharedPtr node); 
protected:
private:
    rclcpp::Node::SharedPtr node_;
};

}

#endif