#include "riptide_navigation/orthogonal_controller.hpp"
#include "riptide_navigation/Constraint.hpp"

#include <string>
#include <map>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include "riptide_msgs/srv/constraint_array.hpp"


namespace riptide_navigation {

    OrthogonalController::OrthogonalController() : Node("orthogonal_controller") {
        service_constraints_ = this->create_service<riptide_msgs::srv::ConstraintArray>(
            "~/constraints", std::bind(
            &OrthogonalController::constraints, this, std::placeholders::_1,
            std::placeholders::_2, std::placeholders::_3));

        // Initialize a and b
        a_ << 1, 0, 0;
        b_ << 1, 0, 0;

        // Initialize u and v vectors
        u_ = std::vector<Eigen::Vector3d>(0);
        v_ = std::vector<Eigen::Vector3d>(0);

        // Initialize a and b markers
        a_marker = std::make_shared<visualization_msgs::msg::Marker>();
        b_marker = std::make_shared<visualization_msgs::msg::Marker>();

        // Initialize u and v markers
        u_markers = std::vector<visualization_msgs::msg::Marker::SharedPtr>(0);
        v_markers = std::vector<visualization_msgs::msg::Marker::SharedPtr>(0);

        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(std::string(this->get_namespace()) + "/riptide_controller/cmd_vel", 10);

        markers_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/marker_array", 10);

        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            std::string(this->get_namespace()) + "/imu_sensor_broadcaster/imu", 10, std::bind(&OrthogonalController::imu_callback, this, std::placeholders::_1));

        twist_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "~/additionnal_twist", 10, std::bind(&OrthogonalController::twist_callback, this, std::placeholders::_1));
    }

    void OrthogonalController::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // Processing time step
        rclcpp::Time t1 = this->now();
        double dt = (t1 - previous_time_).seconds();
        previous_time_ = t1;

        Eigen::Vector3d wm_ = {
            msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z
        };
        Eigen::Matrix3d R_w = (Skew(-dt * wm_)).exp();

        std::scoped_lock<std::mutex> lock_(m_);

        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        for (const auto &[id, constraint]: constraints_) {
            R = R * Skew(dt / constraint->t() * SkewInv(constraint->Matrix().log())).exp();
        }

        for (const auto &[id, constraint]: constraints_) {
            constraint->Update(R_w, dt);
        }

        // Publishing the wanted angular velocity
        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = add_v_(0);
        twist.linear.y = add_v_(1);
        twist.linear.z = add_v_(2);
        twist.angular.x = -R(2,1) + add_w_(0);
        twist.angular.y = -R(0,2) + add_w_(1);
        twist.angular.z = -R(1,0) + add_w_(2);
        twist_publisher_->publish(twist);

        // Manging markers
        publish_markers();
        double epsilon = 0.05;
        if (a_.dot(b_) < 1 - epsilon) {
        }
        else {
            // delete_markers();
        }
    }

    void OrthogonalController::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        add_w_(0) = msg->angular.x;
        add_w_(1) = msg->angular.y;
        add_w_(2) = msg->angular.z;
        add_v_(0) = msg->linear.x;
        add_v_(1) = msg->linear.y;
        add_v_(2) = msg->linear.z;

        RCLCPP_DEBUG(this->get_logger(), "[OrthogonalController] Got v=[%f, %f, %f], w=[%f, %f, %f]",
            add_v_(0), add_v_(1), add_v_(2), add_w_(0), add_w_(1), add_w_(2));
    }

    void OrthogonalController::constraints(
        const std::shared_ptr<rmw_request_id_t> /*request_header*/,
        const std::shared_ptr<riptide_msgs::srv::ConstraintArray::Request> request,
        std::shared_ptr<riptide_msgs::srv::ConstraintArray::Response> /*response*/) {

        std::scoped_lock<std::mutex> lock_(m_);

        for (const auto &c: request->constraints) {
            if (c.type == riptide_msgs::msg::Constraint::DELETE_ALL) {
                constraints_.clear();
                break;
            }
            else if (c.type == riptide_msgs::msg::Constraint::DELETE) {
                auto it = constraints_.find(c.id);
                constraints_.erase(it);
            }
            else {
                // Create FramedVector
                Eigen::Vector3d u_e = {c.u.vector.x, c.u.vector.y, c.u.vector.z};
                Eigen::Vector3d v_e = {c.v.vector.x, c.v.vector.y, c.v.vector.z};
                FramedVector u(u_e, c.u.in_world_frame);
                FramedVector v(v_e, c.v.in_world_frame);

                if (c.type == riptide_msgs::msg::Constraint::ORTHOGONAL) {
                    auto constraint = std::make_shared<OrthogonalConstraint>(c.t, u, v);
                    constraints_[c.id] = constraint;
                }
                else if (c.type == riptide_msgs::msg::Constraint::COLLINEAR) {
                    auto constraint = std::make_shared<OrthogonalConstraint>(c.t, u, v);
                    constraints_[c.id] = constraint;
                }
            }
        }
    }

    // void OrthogonalController::collinear(
    //     const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    //     const std::shared_ptr<riptide_msgs::srv::CollinearConstraint::Request> request,
    //     std::shared_ptr<riptide_msgs::srv::CollinearConstraint::Response> /*response*/) {
        
    //     std::scoped_lock<std::mutex> lock_(m_);
        
    //     geometry_msgs::msg::Vector3 msg_u = request->constraint.u;
    //     a_ = Eigen::Vector3d(msg_u.x, msg_u.y, msg_u.z);
    //     geometry_msgs::msg::Vector3 msg_v = request->constraint.v;
    //     b_ = Eigen::Vector3d(msg_v.x, msg_v.y, msg_v.z);

    //     // Setting the reorientation flag
    //     reorientation_in_progress = true;

    //     // Getting the current time
    //     previous_time_ = this->now();

    //     // Initialize markers
    //     init_markers(0);

    //     // Publishing markers
    //     delete_markers();
    //     publish_markers();

    //     // Info logging
    //     RCLCPP_INFO(this->get_logger(), "[OrthogonalController] Collinear constraint a=[%f, %f, %f], b=[%f, %f, %f]",
    //     a_(0), a_(1), a_(2), b_(0), b_(1), b_(2));
    // }

    // void OrthogonalController::orthogonal(
    //     const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    //     const std::shared_ptr<riptide_msgs::srv::OrthogonalConstraintArray::Request> request,
    //     std::shared_ptr<riptide_msgs::srv::OrthogonalConstraintArray::Response> /*response*/) {
        
    //     std::scoped_lock<std::mutex> lock_(m_);
    //     RCLCPP_INFO(this->get_logger(), "[OrthogonalController] Orthogonal constraint");

    //     // Number of orthogonal constraints
    //     std::size_t n = request->constraints.size();

    //     // Initialize markers
    //     init_markers(n);

    //     // Filling u_
    //     u_.clear();
    //     for (std::size_t i = 0; i < n; ++i) {
    //         geometry_msgs::msg::Vector3 msg_u = request->constraints[i].u;
    //         Eigen::Vector3d u(msg_u.x, msg_u.y, msg_u.z);
    //         u.normalize();
    //         u_.push_back(u);
    //     }

    //     // Filling v_
    //     v_.clear();
    //     for (std::size_t i = 0; i < n; ++i) {
    //         geometry_msgs::msg::Vector3 msg_v = request->constraints[i].v;
    //         Eigen::Vector3d v(msg_v.x, msg_v.y, msg_v.z);
    //         v.normalize();
    //         v_.push_back(v);
    //     }

    //     switch (n) {
    //         case 1: {
    //             a_ = v_[0];
    //             b_ = v_[0] - u_[0].dot(v_[0]) * u_[0];
    //             b_.normalize();
    //         } break;

    //         case 2: {
    //             if (u_[0] == u_[1]) {
    //                 a_ = v_[0].cross(v_[1]);
    //                 b_ = u_[0];
    //                 a_.normalize();
    //             }
    //             else if (v_[0] == v_[1]) {
    //                 a_ = v_[0];
    //                 b_ = u_[0].cross(u_[1]);
    //                 b_.normalize();
    //             }
    //             // TODO case u1 != u2 and v1 != v2
    //         } break;

    //         case 3: {
    //             a_ = Eigen::Vector3d::Zero();
    //             b_ = Eigen::Vector3d::Zero();
                
    //         } break;

    //         default: {
    //             a_ = Eigen::Vector3d::Zero();
    //             b_ = Eigen::Vector3d::Zero();
    //         }
    //     }

    //     // Setting the reorientation flag
    //     reorientation_in_progress = true;

    //     // Getting the current time
    //     previous_time_ = this->now();

    //     // Publishing markers
    //     delete_markers();
    //     publish_markers();

    //     // Info logging
    //     RCLCPP_INFO(this->get_logger(), "[OrthogonalController] Orthogonal constraint n=%ld, a=[%f, %f, %f], b=[%f, %f, %f]",
    //     n, a_(0), a_(1), a_(2), b_(0), b_(1), b_(2));
    // }

    OrthogonalController::ArrowPtr OrthogonalController::ArrowVector(std_msgs::msg::ColorRGBA color) {
        auto m = std::make_shared<visualization_msgs::msg::Marker>();
        std::string tf_name(this->get_namespace());
        tf_name.erase(0, 1);
        m->header.frame_id = tf_name;
        m->ns = "";
        m->type = visualization_msgs::msg::Marker::ARROW;
        m->scale.x = 0.8;
        m->scale.y = 0.06;
        m->scale.z = 0.06;
        m->color = color;
        return m;
    }

    void OrthogonalController::init_markers(std::size_t n) {
        // Create a marker
        std_msgs::msg::ColorRGBA a_color;
        a_color.a = 1.;
        a_color.r = 120./255.;
        a_color.g = 81./255.;
        a_color.b = 169./255.;
        a_marker = ArrowVector(a_color);
        a_marker->id = 0;

        // Create b marker
        std_msgs::msg::ColorRGBA b_color;
        b_color.a = 1.;
        b_color.r = 65./255.;
        b_color.g = 105./255.;
        b_color.b = 1.;
        b_marker = ArrowVector(b_color);
        b_marker->id = 1;

        // Create u_ markers
        u_markers.clear();
        for (std::size_t i = 0; i < n; ++i) {
            std_msgs::msg::ColorRGBA color; color.a = 1.; color.r = 1.;
            auto m = ArrowVector(color);
            m->id = 2 + i;
            u_markers.push_back(m);
        }

        // Create v_ markers
        v_markers.clear();
        for (std::size_t i = 0; i < n; ++i) {
            std_msgs::msg::ColorRGBA color; color.a = 1.; color.g = 1.;
            auto m = ArrowVector(color);
            m->id = 2*(n+1) + i;
            v_markers.push_back(m);
        }
    }

    geometry_msgs::msg::Pose OrthogonalController::VectorToPose(Eigen::Vector3d v) {
        geometry_msgs::msg::Pose pose;
        Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::Base::UnitX(), v.normalized());
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
        return pose;
    }

    void OrthogonalController::publish_markers() {
        // Marker array msg
        visualization_msgs::msg::MarkerArray marker_array;

        // a marker
        a_marker->header.stamp = this->now();
        a_marker->pose = VectorToPose(a_);
        marker_array.markers.push_back(*a_marker);

        // b marker
        b_marker->header.stamp = this->now();
        b_marker->pose = VectorToPose(b_);
        marker_array.markers.push_back(*b_marker);

        // Updating u_markers
        for (std::size_t i = 0; i < u_markers.size(); ++i) {
            u_markers[i]->header.stamp = this->now();
            u_markers[i]->pose = VectorToPose(u_[i]);
            marker_array.markers.push_back(*u_markers[i]);
        }

        // Updating u_markers
        for (std::size_t i = 0; i < v_markers.size(); ++i) {
            v_markers[i]->header.stamp = this->now();
            v_markers[i]->pose = VectorToPose(v_[i]);
            marker_array.markers.push_back(*v_markers[i]);
        }
      
        // Publish marker_array
        markers_publisher_->publish(marker_array);
    }

    void OrthogonalController::delete_markers() {
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
    rclcpp::spin(std::make_shared<riptide_navigation::OrthogonalController>());
    rclcpp::shutdown();
    return 0;
}