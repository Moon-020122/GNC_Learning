#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "pid.hpp"
#include "kinematics.hpp" //for RigidBodyState
#include "rotation.hpp"   //for quatToRbn

namespace gnc
{

    struct Reference
    {
        Eigen::Vector3d p_des = Eigen::Vector3d::Zero(); // desired position (m)
        Eigen::Vector3d v_ff = Eigen::Vector3d::Zero();  // feedforward velocity (m/s)
        Eigen::Vector3d a_ff = Eigen::Vector3d::Zero();  // feedforward acceleration (m/s^2)
        double yaw_des = 0.0;                            // desired yaw(rad)
    };

    // Controller parameters
    struct CascadeParams
    {
        double mass = 1.0;
        double gravity = 9.81; // positive Down in NED

        // Limits
        double v_max = 3.0;                                       // m/s (norm limit)
        double a_max = 6.0;                                       // m/s^2 (norm limit)
        double tilt_max_rad = 0.6;                                // rad (~34 deg)
        double omega_max = 6.0;                                   // rad/s (norm)  angle velocity
        Eigen::Vector3d tau_max = Eigen::Vector3d(1.0, 1.0, 0.5); // N*m per-axis

        double thrust_min = 0.0;
        double thrust_max = 100.0;

        // Attitude P gain:omega_des = Kp_att * e_R(elementwise)
        Eigen::Vector3d kp_att = Eigen::Vector3d(6.0, 6.0, 3.0);

        // PID controller (vector)
        PidConfig pos_pid_cfg;  // p-> v_correction(3D)
        PidConfig vel_pid_cfg;  // v-> a_correction(3D)
        PidConfig rate_pid_cfg; // omega->tau(3D)

        // 1.pos_pid_cfg  "Distance is 10 meters, recommended speed is 2m/s."

        // 2.vel_pid_cfg  "To increase the speed to 2m/s,
        // it is recommended that the acceleration (inclination angle) be 15 degrees."

        // 3.kp_att       "To reach a 15-degree tilt, it's currently 0 degrees.
        // It's recommended to rotate that way at an angular velocity of 60 degrees per second."

        // 4.rate_pid_cfg "To achieve a rotational speed of 60 degrees per second,
        // the motor needs to output a torque of 0.8 N·m."

        // 5.tau_max      "Wait! 0.8 does not exceed the limit (<1.0), approved for execution!"
    };

    struct CascadeOutput
    {
        double thrust_cmd = 0.0;
        Eigen::Vector3d tau_cmd = Eigen::Vector3d::Zero(); // N*m

        Eigen::Matrix3d R_des = Eigen::Matrix3d::Identity(); // desired Rotation matrix
        Eigen::Vector3d omega_des = Eigen::Vector3d::Zero();

        Eigen::Vector3d v_des = Eigen::Vector3d::Zero();
        Eigen::Vector3d a_des = Eigen::Vector3d::Zero();

        Eigen::Vector4d wrench_cmd = Eigen::Vector4d::Zero(); //[T, tau_x, tau_y, tau_z];
    };

    class CascadeController
    {
    public:
        CascadeController();

        void setParams(const CascadeParams &p);
        const CascadeParams &params() const { return p_; }

        void reset();

        CascadeOutput update(const RigidBodyState &s,
                             const Reference &r,
                             double dt);

    private:
        CascadeParams p_;
        // 3D
        Pid pos_pid_;
        Pid vel_pid_;
        Pid rate_pid_;

        static Eigen::Vector3d clampNorm(const Eigen::Vector3d &v, double vmax);
        static double clamp(double x, double lo, double hi);

        // a_des -> (R_des, T_des)
        /*
         Any "desired acceleration ($a_{des}$)"
         corresponds to a unique "attitude ($R_{des}$)" and "total thrust ($T_{des}$)".
         This function is used to calculate this account.
        */
        void accelToAttitudeThrust(const Eigen::Vector3d &a_des_n,
                                   double yaw_des,
                                   Eigen::Matrix3d *R_des_out,
                                   double *T_des_out) const;
        // attitude error(body) from R and R_{des}
        static Eigen::Vector3d attitudeError(const Eigen::Matrix3d &R_bn,
                                             const Eigen::Matrix3d &r_des);
    };
};