// #include <Eigen/src/Core/Matrix.h>
// #include <cmath>
#include <state-observation/dynamical-system/dynamical-system-functor-base.hpp>
#include <Eigen/Dense>
#include <state-observation/api.h>
#include <state-observation/tools/definitions.hpp>
#include <state-observation/tools/rigid-body-kinematics.hpp>
#include <mc_control/GlobalPlugin.h>

namespace stateObservation
{
    class BirjandiStateDynamics : public DynamicalSystemFunctorBase
    {
    public:
        BirjandiStateDynamics(double dt);

        Vector stateDynamics(const Vector & x, const Vector & u, TimeIndex k) override;
        Vector measureDynamics(const Vector & x, const Vector & u, TimeIndex k) override;
        Index getStateSize() const override; // [q, qdot, qddot, qdddot]
        Index getInputSize() const override; // [3x accel, 3x gyro, 3x prev gyro, 3x ang accel, 3x prev ang accel]
        Index getMeasurementSize() const override; // [q, 3x accel, 3x gyro]

        Vector3 getYaMeasurement();

        bool checkStateVector(const Vector & v) override;
        bool checkInputvector(const Vector & v) override;

        void setSamplingPeriod(double dt);
        void setJointOrientation(const Eigen::Matrix3d & R_joint);
        void setJointPosition(const Vector3 & X_pos);

    private:
        double dt_;  // Time step for updating state
        Vector3 omega;
        Vector3 a;
        Vector3 unit;
        stateObservation::kine::Orientation R_joint_;
        Vector3 X_pos_;
    };
} // namespace stateObservation
