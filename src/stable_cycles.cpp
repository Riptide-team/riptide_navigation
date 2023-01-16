#include "riptide_navigation/stable_cycles.hpp"
#include "riptide_msgs/srv/constraint.hpp"
#include "riptide_msgs/srv/constraint_array.hpp"

#include <string>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <map>
#include <mutex>
#include <thread>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <geometry_msgs/msg/vector3.hpp>


namespace riptide_navigation {

    using namespace std::chrono_literals;

    StableCycles::StableCycles() : Node("stable_cycles"), s{State::S1} {
        // IMU subscription
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            std::string(this->get_namespace()) + "/imu_sensor_broadcaster/imu",
            10, std::bind(&StableCycles::imu_callback, this, std::placeholders::_1)
        );

        // Range subscription 
        range_subscription_ = this->create_subscription<sensor_msgs::msg::Range>(
            std::string(this->get_namespace()) + "/echosounder_controller/range", 10,
            std::bind(&StableCycles::range_callback, this, std::placeholders::_1)
        );

        // Additionnal twist publisher
        additionnal_twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            std::string(this->get_namespace()) + "/orthogonal_controller/additionnal_twist", 10
        );

        additionnal_twist_timer_ = this->create_wall_timer(
            500ms, std::bind(&StableCycles::additionnal_twist_callback, this));

        // Orthogonal controller client
        client = this->create_client<riptide_msgs::srv::ConstraintArray>(
            std::string(this->get_namespace()) + "/orthogonal_controller/constraints"
        );

        thread_ = std::thread(std::bind(&StableCycles::loop, this));
    }

    void StableCycles::loop() {
        while (true) {
            if (s == State::S4) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[StableCycles] m1=%f, m2=%f, m3=%f", m1, m2, m3);
            }

            // Execute the method associated to the side
            Methods[s]();

            // Triggering the next state
            s = Transitions[s];
        }
    }

    void StableCycles::additionnal_twist_callback() {
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[StableCycles] Publishing twist!");
        geometry_msgs::msg::Twist twist;
        twist.linear.x = 1;
        additionnal_twist_publisher_->publish(twist);
    }

    void StableCycles::RotateUV(Eigen::Vector3d /*u*/, Eigen::Vector3d /*v*/) {
        // Collinear constraint
        auto request = std::make_shared<riptide_msgs::srv::ConstraintArray::Request>();

        // auto constraint = riptide_msgs::msg::Constraint();
        // constraint.id = 0;
        // constraint.t = 1;
        // constraint.type = riptide_msgs::msg::Constraint::COLLINEAR;
        // // Creating u
        // auto u_fv = riptide_msgs::msg::FramedVector();
        // u_fv.in_world_frame = true;
        // auto u_vec = geometry_msgs::msg::Vector3();
        // u_vec.x = u(0);
        // u_vec.y = u(1);
        // u_vec.z = u(2);
        // u_fv.vector = u_vec;
        // constraint.u = u_fv;

        // // Creating v
        // auto v_fv = riptide_msgs::msg::FramedVector();
        // v_fv.in_world_frame = true;
        // auto v_vec = geometry_msgs::msg::Vector3();
        // v_vec.x = v(0);
        // v_vec.y = v(1);
        // v_vec.z = v(2);
        // v_fv.vector = v_vec;
        // constraint.v = u_fv;
        // request->constraints.push_back(constraint);

        // Send the request
        auto result = client->async_send_request(request);
    }

    void StableCycles::State1() {
        std::chrono::duration<double> duration(d1/3);
        std::this_thread::sleep_for(duration);
        {
            std::scoped_lock<std::mutex> guard(r_mutex_);
            m1 = range_;
        }
        std::this_thread::sleep_for(duration);
        {
            std::scoped_lock<std::mutex> guard(r_mutex_);
            m2 = range_;
        }
        std::this_thread::sleep_for(duration);

        // 90° left turn
        RotateUV(Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitX());
        std::this_thread::sleep_for(1s);
    }

    void StableCycles::State2() {
        std::chrono::duration<double> duration(d2/2);
        std::this_thread::sleep_for(duration);
        {
            std::scoped_lock<std::mutex> guard(r_mutex_);
            m3 = range_;
        }
        std::this_thread::sleep_for(duration);

        // 90° left turn
        RotateUV(Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitX());
        std::this_thread::sleep_for(1s);
    }

    void StableCycles::State3() {
        std::chrono::duration<double> duration(d3);
        std::this_thread::sleep_for(duration);
        // 90° left turn
        RotateUV(Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitX());
    }

    void StableCycles::State4() {
        std::chrono::duration<double> duration(d4);
        std::this_thread::sleep_for(duration);
        // 90° left turn
        RotateUV(Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitX());
        std::this_thread::sleep_for(1s);
    }

    // Imu callback
    void StableCycles::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        std::scoped_lock<std::mutex> guard(a_mutex_);
        am_(0) = msg->linear_acceleration.x;
        am_(1) = msg->linear_acceleration.y;
        am_(2) = msg->linear_acceleration.z;
        am_.normalize();
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[StableCycles] Got am =  %f, %f, %f", am_(0), am_(1), am_(2));
    }

    // Range callback
    void StableCycles::range_callback(const sensor_msgs::msg::Range::SharedPtr msg) {
        std::scoped_lock<std::mutex> guard(r_mutex_);
        range_ = msg->range;
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[StableCycles] Got %f", range_);
    }

} // riptide_navigation

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<riptide_navigation::StableCycles>();
    rclcpp::spin(node);
   
    // rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}