/**
 * This node subscribes to robot description and performs various operations using Pinocchio:
 * - Creates a Pinocchio model from URDF
 * - Performs forward kinematics to get transforms
 * - Computes Jacobians at specific frames
 * - Checks for collisions between links
 */

 #include <memory>
 #include <string>
 #include <vector>
 
 #include "rclcpp/rclcpp.hpp"
 #include "std_msgs/msg/string.hpp"
 #include "sensor_msgs/msg/joint_state.hpp"
 #include "geometry_msgs/msg/transform_stamped.hpp"
 #include "tf2_ros/transform_broadcaster.h"
 
 // Pinocchio headers
 #include "pinocchio/parsers/urdf.hpp"
 #include "pinocchio/algorithm/joint-configuration.hpp"
 #include "pinocchio/algorithm/kinematics.hpp"
 #include "pinocchio/algorithm/jacobian.hpp"
 #include "pinocchio/algorithm/frames.hpp"
 #include "pinocchio/collision/fcl.hpp"
 #include "pinocchio/algorithm/collision.hpp"
 
 class RobotModelNode : public rclcpp::Node {
 public:
   // Constructor
   RobotModelNode() : Node("robot_model_node") {
     // Declare parameters
     this->declare_parameter("urdf_param_name", "robot_description");
     this->declare_parameter("base_link", "base_link");
     this->declare_parameter("end_effector_link", "tool0");
     this->declare_parameter("collision_link1", "link1");
     this->declare_parameter("collision_link2", "link6");
     
     // Get parameters
     urdf_param_name_ = this->get_parameter("urdf_param_name").as_string();
     base_link_ = this->get_parameter("base_link").as_string();
     end_effector_link_ = this->get_parameter("end_effector_link").as_string();
     collision_link1_ = this->get_parameter("collision_link1").as_string();
     collision_link2_ = this->get_parameter("collision_link2").as_string();
     
     // Create subscribers
     urdf_sub_ = this->create_subscription<std_msgs::msg::String>(
       urdf_param_name_, 10, 
       std::bind(&RobotModelNode::urdf_callback, this, std::placeholders::_1));
     
     joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
       "joint_states", 10, 
       std::bind(&RobotModelNode::joint_state_callback, this, std::placeholders::_1));
     
     // Create transform broadcaster
     tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
     
     RCLCPP_INFO(this->get_logger(), "Robot Model Node initialized");
   }
 
 private:
   // Callback for URDF subscription
   void urdf_callback(const std_msgs::msg::String::SharedPtr msg) {
     RCLCPP_INFO(this->get_logger(), "Received robot description, creating Pinocchio model");
     
     // Create Pinocchio model
     model_ = std::make_shared<pinocchio::Model>();
     
     // Parse URDF string to Pinocchio model
     try {
       // This is a test function that parses a URDF string and creates a Pinocchio model
       pinocchio::urdf::buildModelFromXML(msg->data, *model_);
       RCLCPP_INFO(this->get_logger(), "Successfully created Pinocchio model with %ld DoF", model_->nq);
       
       // Create data for the model
       data_ = std::make_shared<pinocchio::Data>(*model_);
       
       // Find frame IDs
       try {
         ee_frame_id_ = model_->getFrameId(end_effector_link_);
         RCLCPP_INFO(this->get_logger(), "Found end effector frame: %s (ID: %ld)", 
                    end_effector_link_.c_str(), ee_frame_id_);
       } catch (const std::exception& e) {
         RCLCPP_ERROR(this->get_logger(), "Failed to find end effector frame: %s", e.what());
       }
       
       // Set up collision model
       collision_model_ = std::make_shared<pinocchio::GeometryModel>();
       
       // For a real implementation, we would parse collision geometries from URDF
       // Here we'll create a simple collision checking setup
       setupCollisionModel();
       
       // Set the model as initialized
       model_initialized_ = true;
       
       // Run some tests if we have joint states already
       if (last_joint_state_) {
         processJointState(last_joint_state_);
       }
       
     } catch (const std::exception& e) {
       RCLCPP_ERROR(this->get_logger(), "Failed to build model from URDF: %s", e.what());
     }
   }
   
   // Set up a simple collision model for demonstration
   void setupCollisionModel() {
     RCLCPP_INFO(this->get_logger(), "Setting up collision model");
     
     // In a real implementation, we'd load this from the URDF
     // For demonstration, we're creating simple collision geometries
     try {
       collision_data_ = std::make_shared<pinocchio::GeometryData>(*collision_model_);
       RCLCPP_INFO(this->get_logger(), "Collision model created successfully");
     } catch (const std::exception& e) {
       RCLCPP_ERROR(this->get_logger(), "Failed to set up collision model: %s", e.what());
     }
   }
   
   // Callback for joint state messages
   void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
     // Store the joint state for later use
     last_joint_state_ = msg;
     
     // If model is not yet initialized, we can't process the joint state
     if (!model_initialized_) {
       RCLCPP_INFO_ONCE(this->get_logger(), 
                       "Received joint state but model not yet initialized. Waiting for robot description.");
       return;
     }
     
     processJointState(msg);
   }
   
   // Process joint states with the Pinocchio model
   void processJointState(const sensor_msgs::msg::JointState::SharedPtr msg) {
     RCLCPP_INFO(this->get_logger(), "Processing joint state");
     
     // Extract joint positions
     Eigen::VectorXd q = Eigen::VectorXd::Zero(model_->nq);
     
     // Map joint states to the model's joint order
     for (size_t i = 0; i < msg->name.size(); i++) {
       // Find this joint in the model
       const std::string& joint_name = msg->name[i];
       
       try {
         // Check if this joint exists in our model
         if (model_->existJointName(joint_name)) {
           // Get joint ID
           const pinocchio::JointIndex joint_id = model_->getJointId(joint_name);
           
           // Get position index for this joint
           const int q_idx = model_->joints[joint_id].idx_q();
           
           // If the joint has a single DOF (most common case for revolute joints)
           if (joint_id < model_->joints.size() && 
               model_->joints[joint_id].nq() == 1) {
             q[q_idx] = msg->position[i];
           }
         }
       } catch (const std::exception& e) {
         RCLCPP_WARN(this->get_logger(), "Error processing joint %s: %s", 
                    joint_name.c_str(), e.what());
       }
     }
     
     // Test 1: Perform forward kinematics
     performForwardKinematics(q);
     
     // Test 2: Get Jacobian at end effector
     computeJacobian(q);
     
     // Test 3: Check collisions
     checkCollisions(q);
   }
   
   // Test 1: Perform forward kinematics and get transforms
   void performForwardKinematics(const Eigen::VectorXd& q) {
     RCLCPP_INFO(this->get_logger(), "Test 1: Performing forward kinematics");
     
     try {
       // Compute forward kinematics
       pinocchio::forwardKinematics(*model_, *data_, q);
       
       // Compute all frames (updates frame placements)
       pinocchio::updateFramePlacements(*model_, *data_);
       
       // Get end effector transform if the frame exists
       if (model_->existFrame(end_effector_link_)) {
         const pinocchio::SE3& ee_transform = data_->oMf[ee_frame_id_];
         
         // Convert to ROS transform message
         geometry_msgs::msg::TransformStamped transform_msg;
         transform_msg.header.stamp = this->now();
         transform_msg.header.frame_id = base_link_;
         transform_msg.child_frame_id = end_effector_link_;
         
         // Translation
         transform_msg.transform.translation.x = ee_transform.translation()[0];
         transform_msg.transform.translation.y = ee_transform.translation()[1];
         transform_msg.transform.translation.z = ee_transform.translation()[2];
         
         // Rotation (convert to quaternion)
         Eigen::Quaterniond quat(ee_transform.rotation());
         transform_msg.transform.rotation.x = quat.x();
         transform_msg.transform.rotation.y = quat.y();
         transform_msg.transform.rotation.z = quat.z();
         transform_msg.transform.rotation.w = quat.w();
         
         // Broadcast transform
         tf_broadcaster_->sendTransform(transform_msg);
         
         RCLCPP_INFO(this->get_logger(), "End effector position: [%f, %f, %f]",
                    ee_transform.translation()[0],
                    ee_transform.translation()[1],
                    ee_transform.translation()[2]);
       } else {
         RCLCPP_WARN(this->get_logger(), "End effector frame '%s' not found in model", 
                    end_effector_link_.c_str());
       }
     } catch (const std::exception& e) {
       RCLCPP_ERROR(this->get_logger(), "Error in forward kinematics: %s", e.what());
     }
   }
   
   // Test 2: Compute Jacobian at specific frame
   void computeJacobian(const Eigen::VectorXd& q) {
     RCLCPP_INFO(this->get_logger(), "Test 2: Computing Jacobian at end effector");
     
     try {
       if (model_->existFrame(end_effector_link_)) {
         // Compute the Jacobian
         Eigen::MatrixXd J(6, model_->nv);
         J.setZero();
         
         // Compute the frame Jacobian
         pinocchio::computeFrameJacobian(*model_, *data_, q, ee_frame_id_, 
                                         pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);
         
         // Print the first few rows and columns of the Jacobian for demonstration
         RCLCPP_INFO(this->get_logger(), "Jacobian (first 3x3 block):");
         for (int i = 0; i < std::min(3, (int)J.rows()); i++) {
           std::stringstream ss;
           for (int j = 0; j < std::min(3, (int)J.cols()); j++) {
             ss << J(i, j) << " ";
           }
           RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
         }
       } else {
         RCLCPP_WARN(this->get_logger(), "End effector frame '%s' not found in model", 
                    end_effector_link_.c_str());
       }
     } catch (const std::exception& e) {
       RCLCPP_ERROR(this->get_logger(), "Error computing Jacobian: %s", e.what());
     }
   }
   
   // Test 3: Check for collisions between links
   void checkCollisions(const Eigen::VectorXd& q) {
     RCLCPP_INFO(this->get_logger(), "Test 3: Checking collisions between links");
     
     try {
       // In a real implementation, we would do:
       // pinocchio::computeCollisions(*model_, *data_, *collision_model_, *collision_data_, q);
       
       // For demonstration purposes, we'll just check if the links exist and report distance
       if (model_->existFrame(collision_link1_) && model_->existFrame(collision_link2_)) {
         // Get frame IDs
         pinocchio::FrameIndex frame1_id = model_->getFrameId(collision_link1_);
         pinocchio::FrameIndex frame2_id = model_->getFrameId(collision_link2_);
         
         // Get positions of frames after forward kinematics
         const Eigen::Vector3d& pos1 = data_->oMf[frame1_id].translation();
         const Eigen::Vector3d& pos2 = data_->oMf[frame2_id].translation();
         
         // Calculate simple Euclidean distance between frames
         double distance = (pos1 - pos2).norm();
         
         RCLCPP_INFO(this->get_logger(), "Distance between '%s' and '%s' is %f meters",
                    collision_link1_.c_str(), collision_link2_.c_str(), distance);
                    
         // In a real collision checker, we would use the actual geometries
         bool collision = distance < 0.1; // Simple threshold for demo
         
         if (collision) {
           RCLCPP_WARN(this->get_logger(), "Potential collision detected between links!");
         } else {
           RCLCPP_INFO(this->get_logger(), "No collision detected between links");
         }
       } else {
         RCLCPP_WARN(this->get_logger(), "Collision links not found in model");
       }
     } catch (const std::exception& e) {
       RCLCPP_ERROR(this->get_logger(), "Error checking collisions: %s", e.what());
     }
   }
   
   // Member variables
   std::string urdf_param_name_;
   std::string base_link_;
   std::string end_effector_link_;
   std::string collision_link1_;
   std::string collision_link2_;
   
   // Pinocchio model and data
   std::shared_ptr<pinocchio::Model> model_;
   std::shared_ptr<pinocchio::Data> data_;
   std::shared_ptr<pinocchio::GeometryModel> collision_model_;
   std::shared_ptr<pinocchio::GeometryData> collision_data_;
   
   // Subscribers
   rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
   rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
   
   // Transform broadcaster
   std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
   
   // Cached joint state
   sensor_msgs::msg::JointState::SharedPtr last_joint_state_;
   
   // Frame IDs
   pinocchio::FrameIndex ee_frame_id_;
   
   // State flag
   bool model_initialized_ = false;
 };
 
 int main(int argc, char * argv[]) {
   rclcpp::init(argc, argv);
   auto node = std::make_shared<RobotModelNode>();
   RCLCPP_INFO(node->get_logger(), "Starting Robot Model Node");
   rclcpp::spin(node);
   rclcpp::shutdown();
   return 0;
 }