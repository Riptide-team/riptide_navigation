#include "riptide_msgs/srv/orthogonal_constraint.hpp"
#include "riptide_msgs/srv/orthogonal_constraint_array.hpp"

#include <string>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <mutex>

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"


namespace riptide_navigation {

    using namespace std::chrono_literals;

    class StateMachine : public rclcpp::Node {
        public:

            using RequestPtr = std::shared_ptr<riptide_msgs::srv::OrthogonalConstraintArray::Request>;

            StateMachine() : Node("state_machine") {
                Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI_4 / 2., Eigen::Vector3d::UnitZ()));
                R_ = q.matrix();

                this->declare_parameter("mode", "plane_rotation");

                twist_msg_ = std::make_shared<geometry_msgs::msg::Twist>();

                client = this->create_client<riptide_msgs::srv::OrthogonalConstraintArray>("/orthogonal_controller/reorient");
                service_timer_ = this->create_wall_timer(5s, std::bind(&StateMachine::service_call, this));

                imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
                    "/imu_sensor_broadcaster/imu", 10, std::bind(&StateMachine::imu_callback, this, std::placeholders::_1));

                // Additionnal twist publisher
                additionnal_twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/orthogonal_controller/additionnal_twist", 10);
                // twist_timer_ = this->create_wall_timer(20ms, std::bind(&StateMachine::twist_callback, this));
            }

        private:
            rclcpp::TimerBase::SharedPtr service_timer_;
            rclcpp::TimerBase::SharedPtr twist_timer_;

            rclcpp::Client<riptide_msgs::srv::OrthogonalConstraintArray>::SharedPtr client;

            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr additionnal_twist_publisher_;

            geometry_msgs::msg::Twist::SharedPtr twist_msg_;

            Eigen::Matrix3d R_;

            unsigned int counter;

            // Imu subscriber
            rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;

            Eigen::Vector3d am_;

            std::mutex a_mutex_;

            // Imu callback
            void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
                std::lock_guard<std::mutex> guard(a_mutex_);
                am_(0) = msg->linear_acceleration.x;
                am_(1) = msg->linear_acceleration.y;
                am_(2) = msg->linear_acceleration.z;
                am_.normalize();
            }

            void twist_callback() {
                twist_msg_->angular.x = 0.5;
                additionnal_twist_publisher_->publish(*twist_msg_);
            }

            RequestPtr one_constraint() {
                // Orthogonal constraint array
                auto request = std::make_shared<riptide_msgs::srv::OrthogonalConstraintArray::Request>();

                // Orthogonal constraint
                riptide_msgs::msg::OrthogonalConstraint o;
                Eigen::Vector3d u = R_ * Eigen::Vector3d::Base::UnitX();
                o.u.x = u(0); o.u.y = u(1); o.u.z = u(2);
                o.v.x = 1.; o.v.y = 0.; o.v.z = 0.;
                request->constraints.push_back(o);

                return request;
            }

            RequestPtr one_constraint_z() {
                // Orthogonal constraint array
                auto request = std::make_shared<riptide_msgs::srv::OrthogonalConstraintArray::Request>();

                // Orthogonal constraint
                std::lock_guard<std::mutex> guard(a_mutex_);
                riptide_msgs::msg::OrthogonalConstraint o;
                o.u.x = am_(0); o.u.y = am_(1); o.u.z = am_(2);
                o.v.x = 1.; o.v.y = 0.; o.v.z = 0.;
                request->constraints.push_back(o);

                return request;
            }

            RequestPtr one_constraint_vertical() {
                // Orthogonal constraint array
                auto request = std::make_shared<riptide_msgs::srv::OrthogonalConstraintArray::Request>();

                // Orthogonal constraint
                riptide_msgs::msg::OrthogonalConstraint o;
                Eigen::Vector3d u = {1, 0, 0.2};
                u.normalize();
                o.u.x = u(0); o.u.y = u(1); o.u.z = u(2);
                o.v.x = 1.; o.v.y = 0.; o.v.z = 0.;
                request->constraints.push_back(o);

                return request;
            }

            RequestPtr two_constraints() {
                // Orthogonal constraint array
                auto request = std::make_shared<riptide_msgs::srv::OrthogonalConstraintArray::Request>();

                for (int i=0; i<2; ++i) {
                    riptide_msgs::msg::OrthogonalConstraint o;
                    std::lock_guard<std::mutex> guard(a_mutex_);
                    o.u.x = am_(0); o.u.y = am_(1); o.u.z = am_(2);
                    o.v.x = (i==0) ? 1. : 0.;
                    o.v.y = (i==1) ? 1. : 0.;
                    o.v.z = 0.;
                    request->constraints.push_back(o);
                }

                return request;
            }

            void service_call() {
                // Orthogonal constraint array
                RequestPtr request;

                // Getting the number of contraints from parameters.
                std::string  mode = this->get_parameter("mode").get_parameter_value().get<std::string>();
                if (mode == "plane") {
                    if (counter%2 == 0) {
                    request = one_constraint_vertical();
                    }
                    else {
                        request = one_constraint_z();
                    }
                    counter++;
                }
                else if (mode == "plane_rotation") {
                    if (counter == 0) {
                        request = one_constraint_vertical();
                    }
                    else if(counter == 1) {
                        request = one_constraint_z();
                        twist_timer_ = this->create_wall_timer(20ms, std::bind(&StateMachine::twist_callback, this));
                    }
                    else {
                        request = one_constraint_z();
                    }
                    counter++;
                }
                else if (mode == "one") {
                    request = one_constraint();
                }
                else if (mode == "one_rotation") {
                    if (counter == 0) {
                        twist_timer_ = this->create_wall_timer(20ms, std::bind(&StateMachine::twist_callback, this));
                    }
                    request = one_constraint();
                    counter++;
                }
                else if (mode == "two") {
                    request = two_constraints();
                }
                
                // Send request
                while (!client->wait_for_service(1s)) {
                    if (!rclcpp::ok()) {
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[StateMachine] Interrupted while waiting for the service.");
                        rclcpp::shutdown();
                    }
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[StateMachine] service not available, waiting again...");
                }

                auto result = client->async_send_request(request, std::bind(&StateMachine::response_callback, this, std::placeholders::_1));
            };

            void response_callback(rclcpp::Client<riptide_msgs::srv::OrthogonalConstraintArray>::SharedFuture future) {
                auto status = future.wait_for(1s);
                if (status == std::future_status::ready) {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[StateMachine] Successfully called the service");
                }
                else {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[StateMachine] Service call in progress ...");
                }
            };
    };
} // riptide_navigation

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<riptide_navigation::StateMachine>());
    rclcpp::shutdown();
    return 0;
}