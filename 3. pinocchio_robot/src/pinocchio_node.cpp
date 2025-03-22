#include <rclcpp/rclcpp.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/collision.hpp>
#include "kinematics.hpp"
#include "jacobian.hpp"
#include "collision_check.hpp"

class PinocchioNode : public rclcpp::Node {
public:
    PinocchioNode() : Node("pinocchio_node") {
        RCLCPP_INFO(this->get_logger(), "Starting Pinocchio Node...");

        pinocchio::Model model;
        pinocchio::Data data(model);

        // Load URDF
        std::string urdf_path = "urdf/robot.urdf"; 
        pinocchio::urdf::buildModel(urdf_path, model);

        // Forward Kinematics Test
        Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
        performForwardKinematics(model, data, q);

        // Jacobian Calculation Test
        computeJacobian(model, data, q, "link_3");

        // Collision Check Test
        checkCollision(model);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PinocchioNode>());
    rclcpp::shutdown();
    return 0;
}
