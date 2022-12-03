#ifndef NODE_MANAGER_HPP_
#define NODE_MANAGER_HPP_

#include "rclcpp/rclcpp.hpp"
// #include "arpis_ne"

namespace arpis_client {

class NodeManager {
public:
    NodeManager(rclcpp::Node::SharedPtr node1, rclcpp::Node::SharedPtr node2); 
protected:
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Node::SharedPtr node2_;
};

}

#endif