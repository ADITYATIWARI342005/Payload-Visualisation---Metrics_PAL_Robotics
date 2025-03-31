#ifndef ROS2_PINOCCHIO_NODE_HPP_  
#define ROS2_PINOCCHIO_NODE_HPP_  

#include <memory>    // For smart pointers  
#include <string>    // For string handling  

#include "rclcpp/rclcpp.hpp"             // ROS 2 C++ client library  
#include "std_msgs/msg/string.hpp"        // Standard message types  
#include "sensor_msgs/msg/joint_state.hpp" // Message type for joint states  

// Include Pinocchio library headers for robot modeling and calculations  
#include "pinocchio/parsers/urdf.hpp"       // For parsing URDF files  
#include "pinocchio/algorithm/joint-configuration.hpp" // For joint configuration  
#include "pinocchio/algorithm/kinematics.hpp"  // For kinematic computations  
#include "pinocchio/algorithm/dynamics.hpp"   // For dynamic computations  

// Use standard namespace for convenience  
using namespace std;  

namespace ros2_pinocchio_node  
{  

class PinocchioNode : public rclcpp::Node  
{  
public:  
  // Constructor for PinocchioNode  
  explicit PinocchioNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());  

  // Default destructor  
  virtual ~PinocchioNode() = default;  

  // Public method to create a robot model from a URDF string  
  bool createModelFromURDF(const string & urdf_string);  

  // Check if the loaded model is valid  
  bool isModelValid() const;  

  // Get the number of degrees of freedom (DoF) of the model  
  size_t getModelDof() const;  

private:  
  // Callback for handling incoming robot description messages  
  void robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg);  

  // Callback for receiving joint state updates  
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);  

  // Process the robot model using the Pinocchio library  
  void processModel();  

  // ROS subscribers for robot description and joint state topics  
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;  
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;  

  // Variables to hold the Pinocchio model and associated data  
  pinocchio::Model model_;  // The robot model  
  shared_ptr<pinocchio::Data> data_{nullptr}; // Holds computation data related to the model  
  bool model_loaded_{false}; // Flag to indicate if the model has been loaded successfully  
  
  // Topic names for robot description and joint states  
  string robot_description_topic_; // Topic for robot description  
  string joint_state_topic_;        // Topic for joint states  
};  

} 

#endif // ROS2_PINOCCHIO_NODE_HPP_  