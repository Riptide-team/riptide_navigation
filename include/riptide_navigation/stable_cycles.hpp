#pragma once

#include <string>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <chrono>
#include <map>
#include <mutex>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>

#include "riptide_msgs/srv/constraint_array.hpp"


namespace riptide_navigation {

    using namespace std::chrono_literals;

    enum State {
        S1,
        S2,
        S3,
        S4
    };

    class StableCycles : public rclcpp::Node, public std::enable_shared_from_this<StableCycles> {
        public:

            StableCycles();

            ~StableCycles() {
                thread_.join();
            };

        private:
            // Thread
            std::thread thread_;

            void loop();

            // State
            State s;

            // State transitions
            std::map<State, State> Transitions = {
                {State::S1, State::S2},
                {State::S2, State::S3},
                {State::S3, State::S4},
                {State::S4, State::S1}
            };

            void RotateUV(Eigen::Vector3d u, Eigen::Vector3d v);

            void State1();
            void State2();
            void State3();
            void State4();

            std::map<State, std::function<void()>> Methods = {
                {State::S1, std::bind(&StableCycles::State1, this)},
                {State::S2, std::bind(&StableCycles::State2, this)},
                {State::S3, std::bind(&StableCycles::State3, this)},
                {State::S4, std::bind(&StableCycles::State4, this)}
            };

            // State x1, x2, x3
            double x1;
            double x2;
            double x3;

            // Measures m1, m2, m3
            double m1;
            double m2;
            double m3;

            // duration
            double d1 = 5;
            double d2 = 5;
            double d3 = 5;
            double d4 = 5;

            rclcpp::Client<riptide_msgs::srv::ConstraintArray>::SharedPtr client;

            // Additionnal twist publisher
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr additionnal_twist_publisher_;

            rclcpp::TimerBase::SharedPtr additionnal_twist_timer_;

            void additionnal_twist_callback();

            Eigen::Matrix3d R_;

            // Imu subscriber
            rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;

            // IMU callback
            void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

            Eigen::Vector3d am_;

            std::mutex a_mutex_;

            // Range subscriber
            rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_subscription_;

            // Range callback
            void range_callback(const sensor_msgs::msg::Range::SharedPtr msg);

            // Measured range
            double range_;

            // Range mutex
            std::mutex r_mutex_;

            // Clock to sleep
            rclcpp::Clock clock_;
    };
} // riptide_navigation