#include "arpis_client/node/client_node.hpp"
#include "arpis_client/model/joint.hpp"

namespace arpis_client {

ClientNode::ClientNode(rclcpp::Node::SharedPtr node)
    : node_(node)
{
}

}