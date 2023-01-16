#pragma once

#include <rclcpp/rclcpp.hpp>
#include "riptide_msgs/srv/constraint_array.hpp"
#include "riptide_navigation/Constraint.hpp"

#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <string>
#include <memory>
#include <mutex>

namespace riptide_navigation {
    /// Returns the 3D cross product Skew symmetric matrix of a given 3D vector.
    template<class Derived>
    inline Eigen::Matrix<typename Derived::Scalar, 3, 3> Skew(const Eigen::MatrixBase<Derived> & vec) {
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
        return (Eigen::Matrix<typename Derived::Scalar, 3, 3>() << 0.0, -vec[2], vec[1],
            vec[2], 0.0, -vec[0], -vec[1], vec[0], 0.0).finished();
    }

    template<class Derived>
    inline Eigen::Vector<typename Derived::Scalar, 3> SkewInv(const Eigen::MatrixBase<Derived> & mat) {
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, 3);
        return (Eigen::Vector<typename Derived::Scalar, 3>() << mat(2, 1), mat(0, 2), mat(1, 0)).finished();
    }

    class OrthogonalController : public rclcpp::Node {
        public:
            using ArrowPtr = std::shared_ptr<visualization_msgs::msg::Marker>;

            OrthogonalController();

        private:
            // Mutex
            std::mutex m_;

            // Inputs
            // Rotation vectors
            Eigen::Vector3d a_;
            Eigen::Vector3d b_;

            // Rotation vectors
            std::vector<Eigen::Vector3d> u_;
            std::vector<Eigen::Vector3d> v_;

            // Measured angular velocity
            Eigen::Vector3d wm_;

            // Previous time
            rclcpp::Time previous_time_;

            // Twist publisher
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;

            // Marker publisher
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_publisher_;

            // Markers
            visualization_msgs::msg::Marker::SharedPtr a_marker;
            visualization_msgs::msg::Marker::SharedPtr b_marker;
            std::vector<visualization_msgs::msg::Marker::SharedPtr> u_markers;
            std::vector<visualization_msgs::msg::Marker::SharedPtr> v_markers;

            // Arrow marker builder
            ArrowPtr ArrowVector(std_msgs::msg::ColorRGBA color);

            // Vector to pose helper
            geometry_msgs::msg::Pose VectorToPose(Eigen::Vector3d v);

            // Initialize markers
            void init_markers(std::size_t n);

            // Publish markers corresponding to u, v and w
            void publish_markers();

            // Publish message to delete markers
            void delete_markers();

            // Imu subscriber
            rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;

            // Imu callback
            void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

            // Additional twist
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscription_;

            // Additionnal twist callback
            void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

            // Additionnal twist
            Eigen::Vector3d add_w_;
            Eigen::Vector3d add_v_;
            
            // Orthogonal constraint
            rclcpp::Service<riptide_msgs::srv::ConstraintArray>::SharedPtr service_constraints_;

            void constraints(
                const std::shared_ptr<rmw_request_id_t> /*request_header*/,
                const std::shared_ptr<riptide_msgs::srv::ConstraintArray::Request> /*req*/,
                std::shared_ptr<riptide_msgs::srv::ConstraintArray::Response> /*res*/
            );

            std::map<std::uint8_t, std::shared_ptr<ConstraintBase>> constraints_;

            // // Collinear constraint
            // rclcpp::Service<riptide_msgs::srv::CollinearConstraint>::SharedPtr service_collinear_;

            // void collinear(
            //     const std::shared_ptr<rmw_request_id_t> /*request_header*/,
            //     const std::shared_ptr<riptide_msgs::srv::CollinearConstraint::Request> /*req*/,
            //     std::shared_ptr<riptide_msgs::srv::CollinearConstraint::Response> /*res*/
            // );
    };
} // riptide_controllers