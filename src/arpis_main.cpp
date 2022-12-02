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
  "right_shoulder_pitch",
  "left_shoulder_pitch",
  "right_shoulder_roll",
  "left_shoulder_roll",
  "right_elbow",
  "left_elbow",
  "right_hip_yaw",
  "left_hip_yaw",
  "right_hip_roll",
  "left_hip_roll",
  "right_hip_pitch",
  "left_hip_pitch",
  "right_knee",
  "left_knee",
  "right_ankle_pitch",
  "left_ankle_pitch",
  "right_ankle_roll",
  "left_ankle_roll",
  "head_pan",
  "head_tilt"
  };

struct arpis_transfer_item {
  int id;
  int position;
};

struct arpis {
  arpis_transfer_item i[22];
};

// typedef arpis_item arp[22];

class ArpisClientNode : public rclcpp::Node {
  public:
  ArpisClientNode(const char *addr, int port): Node("client") {
    tcp = new arpis_network::tcp(addr, port);
    
    tcp->connect();
    auto grp = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options;
    options.callback_group = grp;
    tf_broadcast_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    tcp_receiver_pub = this->create_publisher<tachimawari_interfaces::msg::Joint>("tcp_receiver", 10);
    timer_ = this->create_wall_timer(1ms, std::bind(&ArpisClientNode::exec, this));
    tcp_receiver_sub = this->create_subscription<tachimawari_interfaces::msg::Joint>("tcp_receiver", 10, std::bind(&ArpisClientNode::rviz, this, std::placeholders::_1), options);
    joint_broadcast_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  }
  private:  
  double val2deg(int val) {
    return (val-2048)*180.0/2048;
  }
  void register_joint(std::string name, double pos) {
      sensor_msgs::msg::JointState js;
      js.header.stamp = this->get_clock()->now();
      js.header.frame_id = "world";     
      js.name.push_back(name);
      js.position.push_back(pos);
      joint_broadcast_->publish(js);
  }
  // void rviz_coder(const tachimawari_interfaces::msg::Joint::SharedPtr msg) {
  // }
  void rviz(const tachimawari_interfaces::msg::Joint::SharedPtr msg) {
    geometry_msgs::msg::TransformStamped t;
    
    t.header.set__frame_id("world");
    t.set__child_frame_id("robot"); 
    t.header.stamp = this->get_clock()->now();   
    t.transform.translation.x = 0.5;
    t.transform.translation.y = 0;
    t.transform.translation.z = 0;
    tf_broadcast_->sendTransform(t);         

    // init joint     
    this->register_joint("body_to_robot", (3.14/180)*0);
    this->register_joint("right_shoulder_pitch", (3.14/180)*0);
    this->register_joint("left_shoulder_pitch", (3.14/180)*0);
    this->register_joint("left_hip_yaw", (3.14/180)*0);// switch (msg->id)
    this->register_joint("left_hip_roll", (3.14/180)*0);
    this->register_joint("left_hip_pitch", (3.14/180)*0);
    this->register_joint("left_knee", (3.14/180)*0);
    this->register_joint("left_ankle_pitch", (3.14/180)*0);
    this->register_joint("left_ankle_roll", (3.14/180)*0);        
    this->register_joint("right_hip_yaw", (3.14/180)*0);
    this->register_joint("right_hip_roll", (3.14/180)*0);
    this->register_joint("right_hip_pitch", (3.14/180)*0);
    this->register_joint("right_knee", (3.14/180)*0);
    this->register_joint("right_ankle_pitch", (3.14/180)*0);
    this->register_joint("right_ankle_roll", (3.14/180)*0);    
    this->register_joint("left_shoulder_roll", (3.14/180)*0);
    this->register_joint("left_elbow", (3.14/180)*0);
    this->register_joint("right_shoulder_roll", (3.14/180)*0);
    this->register_joint("right_elbow", (3.14/180)*0);
    this->register_joint("head_pan", (3.14/180)*0);
    this->register_joint("head_tilt", (3.14/180)*0);
        
    // resend joind data with data from server
    this->register_joint(joint_id[msg->id-1], val2deg(msg->position));
    
    RCLCPP_INFO(rclcpp::get_logger("arpis_client"), " id %i : position %f", msg->id, msg->position);
  }

  void exec() {  
    char buffer[1024] = {0};
    tcp->receive(buffer, 1024);
    // if (std::string(buffer).length() != 0)
    char * buff = (char *)buffer;
    // arp * i = (arp *)(void *)buff;
    // arp k = (arp k)&i;
    arpis * items = (arpis *)(void *)buff;
    // RCLCPP_INFO(rclcpp::get_logger("test"), joints.)
        
    for (int i = 0; i < 20; i++) {
      RCLCPP_INFO(rclcpp::get_logger("arpis_client"), "id %i : position %i", (*items).i[i].id, (*items).i[i].position);
      tachimawari_interfaces::msg::Joint joint;
      joint.id = (*items).i[i].id;
      joint.position = (*items).i[i].position;
      this->tcp_receiver_pub->publish(joint);
    }        
  }  
  float tinc;
  rclcpp::Publisher<tachimawari_interfaces::msg::Joint>::SharedPtr tcp_receiver_pub;
  rclcpp::Subscription<tachimawari_interfaces::msg::Joint>::SharedPtr tcp_receiver_sub;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcast_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_broadcast_;
  arpis_network::tcp * tcp;
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
  // auto node = std::make_shared<rclcpp::Node>()
  rclcpp::spin(std::make_shared<ArpisClientNode>(addr, port));
  rclcpp::shutdown();
  
  return 0;
}
