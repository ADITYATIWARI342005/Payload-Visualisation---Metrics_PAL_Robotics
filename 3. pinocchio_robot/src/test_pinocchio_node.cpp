#include <rclcpp/rclcpp.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/collision.hpp>
#include <pinocchio/parsers/urdf.hpp>

class PinocchioTestNode : public rclcpp::Node {
public:
    PinocchioTestNode() : Node("pinocchio_test_node") {
        RCLCPP_INFO(this->get_logger(), "Testing Pinocchio functionalities...");

        std::string urdf_path = "urdf/robot.urdf";
        pinocchio::Model model;
        pinocchio::urdf::buildModel(urdf_path, model);
        pinocchio::Data data(model);

        Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
        pinocchio::forwardKinematics(model, data, q);
        RCLCPP_INFO(this->get_logger(), "Forward kinematics computed successfully.");

        Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, model.nv);
        pinocchio::computeJointJacobian(model, data, q, model.getFrameId("link_3"), jacobian);
        RCLCPP_INFO(this->get_logger(), "Jacobian computed for frame link_3.");

        pinocchio::GeometryModel geom_model;
        pinocchio::GeometryData geom_data(geom_model);
        if (pinocchio::computeCollisions(model, geom_model, geom_data)) {
            RCLCPP_WARN(this->get_logger(), "Collision detected!");
        } else {
            RCLCPP_INFO(this->get_logger(), "No collision detected.");
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PinocchioTestNode>());
    rclcpp::shutdown();
    return 0;
}
