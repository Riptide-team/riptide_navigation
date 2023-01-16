#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <string>
#include <memory>

namespace riptide_navigation {
    /// Returns the 3D cross product Skew symmetric matrix of a given 3D vector.
    template<class Derived>
    inline Eigen::Matrix<typename Derived::Scalar, 3, 3> Skew(const Eigen::MatrixBase<Derived> & vec) {
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
        return (Eigen::Matrix<typename Derived::Scalar, 3, 3>() << 0.0, -vec[2], vec[1],
            vec[2], 0.0, -vec[0], -vec[1], vec[0], 0.0).finished();
    }

    class OrientationController : public rclcpp::Node {
        public:
            OrientationController();

        private:
            // Inputs
            // Rotation vectors
            Eigen::Vector3d ur_;
            Eigen::Vector3d vr_;
            Eigen::Vector3d wr_;

            // Measured angular velocity
            Eigen::Vector3d wm_;

            // Stopping condition
            Eigen::Matrix3d A;

            // Previous time
            rclcpp::Time previous_time_;

            // Twist publisher
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;

            // Marker publisher
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_publisher_;

            // Markers
            visualization_msgs::msg::Marker::SharedPtr u_marker;
            visualization_msgs::msg::Marker::SharedPtr v_marker;
            visualization_msgs::msg::Marker::SharedPtr w_marker;

            // Initialize markers
            void init_markers();

            // Publish markers corresponding to u, v and w
            void publish_markers();

            // Publish message to delete markers
            void delete_markers();

            // Imu subscriber
            rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;

            // Imu callback
            void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
            
            // Reorientation boolean
            bool reorientation_in_progress;

            // Reorient service
            rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_reorient_;

            void reorient(
                const std::shared_ptr<rmw_request_id_t> /*request_header*/,
                const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
                std::shared_ptr<std_srvs::srv::Empty::Response> /*res*/
            );
    };
} // riptide_controllers