#include "jacobian.hpp"

void computeJacobian(const pinocchio::Model &model, pinocchio::Data &data, const Eigen::VectorXd &q, const std::string &frame_name) {
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, model.nv);
    pinocchio::computeJointJacobian(model, data, q, model.getFrameId(frame_name), jacobian);
    RCLCPP_INFO(rclcpp::get_logger("Jacobian"), "Jacobian computed successfully for frame: %s", frame_name.c_str());
}
