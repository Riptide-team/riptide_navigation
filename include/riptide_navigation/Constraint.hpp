#pragma once

#include <eigen3/Eigen/Dense>

namespace riptide_navigation {

    class FramedVector {
        public:
            // Constructor
            FramedVector(Eigen::Vector3d v, bool world_frame=false): v_(v), world_frame_(world_frame) {};

            // Vector getter
            inline Eigen::Vector3d Vector() const { return v_; };

            // Reference frame getter
            inline bool IsWorldFrame() const { return world_frame_; };

            inline void Update(Eigen::Matrix3d R) { v_ = R * v_; };

        private:
            // Vector
            Eigen::Vector3d v_;

            // Is the reference frame the world
            bool world_frame_;
    };
    
    class ConstraintBase {
        public:
            // Constructor
            ConstraintBase(double t, FramedVector u, FramedVector v): t_(t), u_(u), v_(v) {};

            // Update world frame vector
            inline void Update(Eigen::Matrix3d R, double dt) {
                if (u_.IsWorldFrame())
                    u_.Update(R);
                if (v_.IsWorldFrame())
                    v_.Update(R);
                t_ = std::max(0.05, t_ - dt);
            };

            inline double t() const { return t_; };
            inline FramedVector u() const { return u_; };
            inline FramedVector v() const { return v_; };

            virtual Eigen::Matrix3d Matrix() const = 0;

        protected:
            // Transformation duration
            double t_;

            // Constraint first vector
            FramedVector u_;

            // Constraint second vector
            FramedVector v_;
    };

    class CollinearConstraint : public ConstraintBase {
        public:
            // Constructor
            CollinearConstraint(double t, FramedVector u, FramedVector v): ConstraintBase(t, u, v) {};

            // Getting matrix
            inline Eigen::Matrix3d Matrix() const {
                Eigen::Matrix3d K = u_.Vector() * v_.Vector().transpose() - v_.Vector() * u_.Vector().transpose();
                return Eigen::Matrix3d::Identity() + K + 1 / (1 + u_.Vector().dot(v_.Vector())) * K * K;
            }
    };

    class OrthogonalConstraint : public ConstraintBase {
        public:
            // Constructor
            OrthogonalConstraint(double t, FramedVector u, FramedVector v): ConstraintBase(t, u, v) {};

            // Getting matrix
            inline Eigen::Matrix3d Matrix() const {
                Eigen::Vector3d a_ = v_.Vector();
                Eigen::Vector3d b_ = v_.Vector() - u_.Vector().dot(v_.Vector()) * u_.Vector();
                Eigen::Matrix3d K = a_ * b_.transpose() - b_ * a_.transpose();
                return Eigen::Matrix3d::Identity() + K + 1 / (1 + a_.dot(b_)) * K * K;
            }
    };

} // riptide_navigation