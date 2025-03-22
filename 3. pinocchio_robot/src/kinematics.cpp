#include "kinematics.hpp"

void performForwardKinematics(const pinocchio::Model &model, pinocchio::Data &data, const Eigen::VectorXd &q) {
    pinocchio::forwardKinematics(model, data, q);
    RCLCPP_INFO(rclcpp::get_logger("Kinematics"), "Forward kinematics computed successfully.");
}
