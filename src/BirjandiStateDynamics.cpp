#include "BirjandiStateDynamics.h"
#include <state-observation/tools/definitions.hpp>
#include <state-observation/tools/rigid-body-kinematics.hpp>

namespace stateObservation
{
    BirjandiStateDynamics::BirjandiStateDynamics(double dt)
        : dt_(dt)
        {
            omega = Vector3::Zero();
            a = Vector3::Zero();
            X_pos_ = Vector3::Zero();
            R_joint_ = stateObservation::kine::Orientation::zeroRotation();
            unit << 0,1,0;
        }

    Vector BirjandiStateDynamics::stateDynamics(const Vector & x, const Vector & u, TimeIndex k)
    {
        // State vector: [q, qdot, qddot, qdddot]
        double q = x(0);      // Joint position
        double qdot = x(1);   // Joint velocity
        double qddot = x(2);  // Joint acceleration
        double qdddot = x(3); // Joint jerk
        

        // State transition update
        double q_next      = q + qdot*dt_;
        double qdot_next   = qdot + qddot*dt_; // Assuming joint velocity is directly linked to angular acceleration
        double qddot_next  = qddot + qdddot*dt_; // Assuming joint acceleration is directly linked to angular jerk
        double qdddot_next = qdddot; // Assuming constant jerk

        // Return the next state vector
        Vector next_state(getStateSize());
        next_state << q_next, qdot_next, qddot_next, qdddot_next;
        return next_state;
    }

    Vector BirjandiStateDynamics::measureDynamics(const Vector & x, const Vector & u, TimeIndex k)
    {   
        double q = x(0);      // Joint position
        Vector3 q_vec = q*unit;      // Joint position
        Vector3 qdot_vec = x(1)*unit;   // Joint velocity
        Vector3 qddot_vec = x(2)*unit;  // Joint acceleration

        // mc_rtc::log::info("[Birjandi] R_joint: {}", R_joint_.toMatrix3());
        // mc_rtc::log::info("[Birjandi] X_pos: {}", X_pos_);

        a = stateObservation::kine::skewSymmetric(qddot_vec)*X_pos_ + stateObservation::kine::skewSymmetric(qdot_vec)*(stateObservation::kine::skewSymmetric(qdot_vec)*X_pos_);
        // mc_rtc::log::info("[Birjandi] a: {}", a);


        // a = stateObservation::kine::skewSymmetric(qddot_vec)*q_vec + stateObservation::kine::skewSymmetric(qdot_vec)*(stateObservation::kine::skewSymmetric(qdot_vec)*q_vec);
        // acclin_joint_ = R_joint_ * accelero_ - stateObservation::kine::skewSymmetric(angveldot_)*X_pos - stateObservation::kine::skewSymmetric(angvel_joint_)*(stateObservation::kine::skewSymmetric(angvel_joint_)*X_pos);
    
        omega = qdot_vec;

        Vector next_measurement(getMeasurementSize());
        next_measurement << q, a, omega;

        return next_measurement;
    }

    Index BirjandiStateDynamics::getStateSize()const { return 4; }  // [q, qdot, qddot, qdddot]
    Index BirjandiStateDynamics::getInputSize() const { return 0; } // No inputs
    Index BirjandiStateDynamics::getMeasurementSize() const { return 7; } // [q, 3x accel, 3x gyro]

    bool BirjandiStateDynamics::checkStateVector(const Vector & v) { return v.rows() == getStateSize(); }
    bool BirjandiStateDynamics::checkInputvector(const Vector & v) { return v.rows() == getInputSize(); }

    void BirjandiStateDynamics::setSamplingPeriod(double dt) { dt_ = dt; }
    void BirjandiStateDynamics::setJointOrientation(const Eigen::Matrix3d & R_joint) { R_joint_ = R_joint; }
    void BirjandiStateDynamics::setJointPosition(const Vector3 & X_pos) { X_pos_ = X_pos; }

    Vector3 BirjandiStateDynamics::getYaMeasurement() { return a; }
} // namespace stateObservation



        

