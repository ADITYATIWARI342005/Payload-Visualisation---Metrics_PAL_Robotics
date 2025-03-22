#include "collision_check.hpp"

void checkCollision(const pinocchio::Model &model) {
    pinocchio::GeometryModel geom_model;
    pinocchio::GeometryData geom_data(geom_model);

    if (pinocchio::computeCollisions(model, geom_model, geom_data)) {
        RCLCPP_WARN(rclcpp::get_logger("CollisionCheck"), "Collision detected!");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("CollisionCheck"), "No collision detected.");
    }
}
