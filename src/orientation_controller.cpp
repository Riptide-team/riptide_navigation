#include "riptide_navigation/orientation_controller.hpp"

#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <geometry_msgs/msg/pose.hpp>


namespace riptide_navigation {

    OrientationController::OrientationController() : Node("orientation_controller") {
        // Initialize reorientation_in_progress
        reorientation_in_progress = false;

        // Initialize input vectors
        ur_ = Eigen::Vector3d::Base::UnitX();
        vr_ = Eigen::Vector3d::Base::UnitY();
        wr_ = Eigen::Vector3d::Base::UnitZ();
        A << ur_, vr_, wr_;

        init_markers();

        service_reorient_ = this->create_service<std_srvs::srv::Empty>(
            "~/reorient", std::bind(
            &OrientationController::reorient, this, std::placeholders::_1,
            std::placeholders::_2, std::placeholders::_3));

        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/riptide_controller/cmd_vel", 10);

        markers_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/marker_array", 10);

        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu_sensor_broadcaster/imu", 10, std::bind(&OrientationController::imu_callback, this, std::placeholders::_1));
    }

    void OrientationController::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        if (reorientation_in_progress) {
            // Processing time step
            rclcpp::Time t1 = this->now();
            double dt = (t1 - previous_time_).seconds();
            previous_time_ = t1;

            Eigen::Vector3d wm_ = {
                msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z
            };

            // Updating vr_
            Eigen::Matrix3d R = (Skew(dt * wm_)).exp();
            vr_ = R.transpose() * vr_;
            A.block(0, 1, 3, 1) = vr_;

            if (A.determinant() > 0) {
                // Vector to vector formulation https://en.wikipedia.org/wiki/Rotation_matrix#Vector_to_vector_formulation
                ur_.normalize();
                vr_.normalize();
                Eigen::Matrix3d K = ur_ * vr_.transpose() - vr_ * ur_.transpose();
                Eigen::Matrix3d R_ = Eigen::Matrix3d::Identity() + K + 1 / (1 + ur_.dot(vr_)) * K * K;

                // Publishing the wanted angular velocity
                auto twist = geometry_msgs::msg::Twist();
                twist.angular.x = -R_(2,1);
                twist.angular.y = -R_(0,2);
                twist.angular.z = -R_(1,0);

                twist_publisher_->publish(twist);

                publish_markers();
            }
            else {
                // Reorientation ended
                reorientation_in_progress = false;
                delete_markers();
            }
        }
    }

    void OrientationController::reorient(
        const std::shared_ptr<rmw_request_id_t> /*request_header*/,
        const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
        std::shared_ptr<std_srvs::srv::Empty::Response> /*res*/) {
            ur_ = Eigen::Vector3d::Base::UnitX();
            vr_ = Eigen::Vector3d::Base::UnitY() + Eigen::Vector3d::Base::UnitZ();
            wr_ = ur_.cross(vr_);
            A << ur_, vr_, wr_;
            publish_markers();
            reorientation_in_progress = true;
            previous_time_ = this->now();
            RCLCPP_INFO(this->get_logger(), "[OrientationController] Reorient service call");
    }

    void OrientationController::init_markers() {
        // Create markers
        u_marker = std::make_shared<visualization_msgs::msg::Marker>();
        v_marker = std::make_shared<visualization_msgs::msg::Marker>();
        w_marker = std::make_shared<visualization_msgs::msg::Marker>();

        // Initialize u markers
        u_marker->header.frame_id = "base_link";
        u_marker->type = visualization_msgs::msg::Marker::ARROW;
        u_marker->id = 0;
        u_marker->scale.x = 8;
        u_marker->scale.y = 0.6;
        u_marker->scale.z = 0.6;

        std_msgs::msg::ColorRGBA u_colour;
        u_colour.a = 1;
        u_colour.r = 1;
        u_colour.g = 0;
        u_colour.b = 0;
        u_marker->color = u_colour;

        // Initialize v markers
        v_marker->header.frame_id = "base_link";
        v_marker->type = visualization_msgs::msg::Marker::ARROW;
        v_marker->id = 1;
        v_marker->scale.x = 8;
        v_marker->scale.y = 0.6;
        v_marker->scale.z = 0.6;

        std_msgs::msg::ColorRGBA v_colour;
        v_colour.a = 1;
        v_colour.r = 0;
        v_colour.g = 1;
        v_colour.b = 0;
        v_marker->color = v_colour;

        // Initialize w markers
        w_marker->header.frame_id = "base_link";
        w_marker->type = visualization_msgs::msg::Marker::ARROW;
        w_marker->id = 2;
        w_marker->scale.x = 8;
        w_marker->scale.y = 0.6;
        w_marker->scale.z = 0.6;

        std_msgs::msg::ColorRGBA w_colour;
        w_colour.a = 1;
        w_colour.r = 0;
        w_colour.g = 0;
        w_colour.b = 1;
        w_marker->color = w_colour;
    }

    void OrientationController::publish_markers() {
        // u marker
        u_marker->header.stamp = this->now();
        geometry_msgs::msg::Pose u_pose;
        u_marker->pose = u_pose;

        // v marker
        v_marker->header.stamp = this->now();
        geometry_msgs::msg::Pose v_pose;
        // Eigen::Quaterniond q_v(Eigen::AngleAxisd(0., vr_.normalized()));
         Eigen::Quaterniond q_v = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::Base::UnitX(), vr_.normalized());
        v_pose.orientation.x = q_v.x();
        v_pose.orientation.y = q_v.y();
        v_pose.orientation.z = q_v.z();
        v_pose.orientation.w = q_v.w();
        v_marker->pose = v_pose;
        
        // w marker
        w_marker->header.stamp = this->now();
        geometry_msgs::msg::Pose w_pose;
        Eigen::Quaterniond q_w = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::Base::UnitX(), wr_.normalized());
        w_pose.orientation.x = q_w.x();
        w_pose.orientation.y = q_w.y();
        w_pose.orientation.z = q_w.z();
        w_pose.orientation.w = q_w.w();
        w_marker->pose = w_pose;

        visualization_msgs::msg::MarkerArray marker_array;
        marker_array.markers.push_back(*u_marker);
        marker_array.markers.push_back(*v_marker);
        marker_array.markers.push_back(*w_marker);
        markers_publisher_->publish(marker_array);
    }

    void OrientationController::delete_markers() {
        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->now();
        marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(marker);
        markers_publisher_->publish(marker_array);
    }
} // riptide_navigation

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<riptide_navigation::OrientationController>());
    rclcpp::shutdown();
    return 0;
}