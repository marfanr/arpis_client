#include "arpis_client/node/node_manager.hpp"
#include "arpis_client/model/joint.hpp"

namespace arpis_client {

NodeManager::NodeManager(rclcpp::Node::SharedPtr node1, rclcpp::Node::SharedPtr node2)
    : node_(node1), node2_(node2)
{
}

}