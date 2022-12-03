#include <chrono>
#include <iostream>
#include <sys/socket.h>
#include <sys/types.h>
#include <cstring>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "arpis_network/tcp/tcp.hpp"
#include "tachimawari_interfaces/msg/joint.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tachimawari/joint/model/joint.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <vector>


#define _USE_MATH_DEFINES

using namespace std::chrono_literals;

const std::string joint_id[20] = {
  "right_shoulder_pitch", // 1
  "left_shoulder_pitch", // 2
  "right_shoulder_roll", // 3
  "left_shoulder_roll", // 4
  "right_elbow", // 5
  "left_elbow", // 6
  "right_hip_roll", // 7
  "left_hip_roll", // 8
  "right_hip_yaw", // 9
  "left_hip_yaw", // 10
  "right_hip_pitch", // 11
  "left_hip_pitch", // 12
  "right_knee", // 13
  "left_knee", // 14
  "right_ankle_roll", // 15
  "left_ankle_roll", // 16
  "right_ankle_pitch", // 17
  "left_ankle_pitch", // 18
  "head_pan", // 19
  "head_tilt" // 20
  };

float direction[20] = {
    1, // 1
    -1, // 2
    1, // 3
    -1, // 4
    -1, // 5
    1, // 6
    -1, // 7
    -1, // 8
    1, // 9
    -1, // 10
    -1, // 1
    1, // 2
    -1, // 3
    1, // 4
    1, // 5
    1, // 6
    1, // 7
    1, // 8
    1, // 9
    1, // 10
};

struct arpis_transfer_item {
  int id;
  int position;
};

struct arpis {
  arpis_transfer_item i[22];
};

// typedef arpis_item arp[22];

class ArpisClientNode {
  public:
  ArpisClientNode(const char * addr, int port, rclcpp::Node::SharedPtr node, rclcpp::Node::SharedPtr node2)
    : node_(node), node2_(node2) {
    tcp = new arpis_network::tcp(addr, port);
    tcp->connect();

    auto grp = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options;
    // node_->cre
    options.callback_group = grp;
    tf_broadcast_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
    tcp_receiver_pub = node_->create_publisher<tachimawari_interfaces::msg::Joint>("tcp_receiver", 10);
    timer_ = node_->create_wall_timer(1ms, std::bind(&ArpisClientNode::exec, this), grp);
    tcp_receiver_sub = node_->create_subscription<tachimawari_interfaces::msg::Joint>("tcp_receiver", 10, std::bind(&ArpisClientNode::rviz, this, std::placeholders::_1), options);
    joint_broadcast_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  }
  private:  
  double val2deg(int val) {
    // return (val-2048)*180.0/2048 ;
    return (val-2048)*(360.0/4096.0)*(M_PI/180.0);
  }
  void register_joint(sensor_msgs::msg::JointState &js,std::string name, double pos) {
      js.header.frame_id = "world";     
      js.name.push_back(name);
      js.position.push_back(pos);
  }
  // void rviz_coder(const tachimawari_interfaces::msg::Joint::SharedPtr msg) {
  // }
  void rviz(const tachimawari_interfaces::msg::Joint::SharedPtr msg) {
    geometry_msgs::msg::TransformStamped t;
    
    t.header.set__frame_id("world");
    t.set__child_frame_id("robot"); 
    t.header.stamp = node_->get_clock()->now();   
    t.transform.translation.x = 0.5;
    t.transform.translation.y = 0;
    t.transform.translation.z = 0;
    tf_broadcast_->sendTransform(t);         

    // init joint
    this->register_joint(js, "body_to_robot", (3.14/180)*0);
    

    joint_broadcast_->publish(js);
    js.position.clear();
    js.name.clear();
    // RCLCPP_INFO(rclcpp::get_logger("arpis_client"), " id %i : position %f", msg->id, msg->position);
  }

  void exec() {  
    char buffer[1024] = {0};
    // RCLCPP_INFO(rclcpp::get_logger("test"), "s");
    tcp->receive(buffer, 1024);
    // if (std::string(buffer).length() != 0)
    char * buff = (char *)buffer;
    // arp * i = (arp *)(void *)buff;
    // arp k = (arp k)&i;
    arpis * items = (arpis *)(void *)buff;
        
    tachimawari_interfaces::msg::Joint joint;
    for (int i = 0; i < 20; i++) {
      RCLCPP_INFO(rclcpp::get_logger("arpis_client"), "id %i : position %i", (*items).i[i].id, (*items).i[i].position);
      joint.id = (*items).i[i].id;
      joint.position = (*items).i[i].position*direction[joint.id-1];
      register_joint(js, joint_id[joint.id-1], val2deg(joint.position ));
    }        
      js.header.stamp = node_->get_clock()->now();
      this->tcp_receiver_pub->publish(joint);
  }  
  float tinc;
  rclcpp::Publisher<tachimawari_interfaces::msg::Joint>::SharedPtr tcp_receiver_pub;
  rclcpp::Subscription<tachimawari_interfaces::msg::Joint>::SharedPtr tcp_receiver_sub;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcast_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_broadcast_;
  arpis_network::tcp * tcp;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr node2_;
  sensor_msgs::msg::JointState js;

  double position[19];
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const char * addr = "127.0.0.1";
  int port = 3000;
  for (int t = 0; t < argc;t++) {
    if (argv[t] == (std::string)"--addr")
      addr = (const char *)argv[t+1];
    else if (argv[t] == (std::string)"--port")
      port = (int)atoi(argv[t+1]);
  }
  auto node = std::make_shared<rclcpp::Node>("arpis_client_1");
  auto node2 = std::make_shared<rclcpp::Node>("arpis_client_2");

  // rclcpp::spin(std::make_shared<ArpisClientNode>(addr, port));
  rclcpp::executors::SingleThreadedExecutor exec;

  auto client = std::make_shared<ArpisClientNode>(addr, port, node, node2);
  exec.add_node(node);
  // exec.add_node(node2);
  // exec.spin();
  rclcpp::Rate rcl_rate(8ms);
  while (rclcpp::ok())
  {
    rcl_rate.sleep();
    exec.spin_some();
  }

  rclcpp::shutdown();
  
  return 0;
}
