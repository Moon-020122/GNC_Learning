#include "cascade_controller.hpp"
#include <cmath>
#include <algorithm>

namespace gnc
{
    CascadeController::CascadeController()
        : pos_pid_(3), vel_pid_(3), rate_pid_(3) {}

    void CascadeController::setParams(const CascadeParams &p)
    {
        p_ = p;
        pos_pid_.setConfig(p_.pos_pid_cfg);
        vel_pid_.setConfig(p_.vel_pid_cfg);
        rate_pid_.setConfig(p_.rate_pid_cfg);
    }
    // Clear integral terms, historical errors, etc.
    void CascadeController::reset()
    {
        pos_pid_.reset();
        vel_pid_.reset();
        rate_pid_.reset();
    }

    double CascadeController::clamp(double x, double lo, double hi)
    {
        if (x < lo)
            return lo;
        if (x > hi)
            return hi;
        return x;
    }

    Eigen::Vector3d CascadeController::clampNorm(const Eigen::Vector3d &v, double vmax)
    {
        const double n = v.norm();
        if (n <= vmax || n < 1e-12)
            return v;
        return v * (vmax / n);
    }

    Eigen::Vector3d CascadeController::attitudeError(const Eigen::Matrix3d &R_bn,
                                                     const Eigen::Matrix3d &R_des)
    {
        // body-frame rotation error : R_err = R^T * R_des
        Eigen::Matrix3d R_err = R_bn.transpose() * R_des;

        Eigen::AngleAxisd aa(R_err);
        double angle = aa.angle();
        Eigen::Vector3d axis = aa.axis();

        // Ensure numerical safety
        if (!std::isfinite(angle) || !axis.allFinite())
        {
            return Eigen::Vector3d::Zero();
        }

        // e_R = angle * axis (body frame)
        return axis * angle;
    }

    void CascadeController::accelToAttitudeThrust(const Eigen::Vector3d &a_des_n,
                                                  double yaw_des,
                                                  Eigen::Matrix3d *R_des_out,
                                                  double *T_des_out) const
    {
        const Eigen::Vector3d g_n(0.0, 0.0, p_.gravity);
        // from dynamics : a= g- (T/m)*be (because thrust is -T b3)
        //  => b3_des = -(a_des - g)/ ||a_des -g||, T_des = m|| a_des-g||
        Eigen::Vector3d a_tilde = a_des_n - g_n; // u
        const double norm_at = a_tilde.norm();   // ||u||

        Eigen::Vector3d b3_des(0.0, 0.0, 1.0);
        double T_des = p_.mass * p_.gravity;

        if (norm_at > 1e-8 && std::isfinite(norm_at))
        {
            b3_des = -a_tilde / norm_at;
            T_des = p_.mass * norm_at;
        }
        // tilt limit : clamp angle between b3_des and [0,0,1]
        // b3_des is unit; its z component is cos(tilt) if near hover
        // The tilt angle of a drone is the angle between its tilted direction and the vertical axis.
        const double z = b3_des.z();
        const double z_clamped = clamp(z, -1.0, 1.0);
        double tilt = std::acos(z_clamped); // drone tilt  angle (theta)

        if (tilt > p_.tilt_max_rad)
        {
            Eigen::Vector2d xy(b3_des.x(), b3_des.y());
            const double xy_norm = xy.norm();
            if (xy_norm < 1e-9)
            {
                b3_des = Eigen::Vector3d(0.0, 0.0, 1.0); // hover
            }
            else
            {
                const double z_new = std::cos(p_.tilt_max_rad);
                const double xy_new = std::sin(p_.tilt_max_rad);
                Eigen::Vector2d xy_dir = xy / xy_norm; // direction
                b3_des = Eigen::Vector3d(xy_dir.x() * xy_new, xy_dir.y() * xy_new, z_new);
            }
        }

        // Build desired heading reference in NED (projected onto horizontal plane) DESIRED YAW!
        Eigen::Vector3d b1_c(std::cos(yaw_des), std::sin(yaw_des), 0.0);
        // if b3 and b1_c nearly parallel , choose a different b1_c
        if (b3_des.cross(b1_c).norm() < 1e-6)
        {
            b1_c = Eigen::Vector3d(0.0, 1.0, 0.0);
        }
        if (b3_des.cross(b1_c).norm() < 1e-6)
        {
            b1_c = Eigen::Vector3d(1.0, 0.0, 0.0);
        }

        Eigen::Vector3d b2_des = b3_des.cross(b1_c);
        const double n2 = b2_des.norm();
        if (n2 < 1e-9)
        {
            b2_des = Eigen::Vector3d(0.0, 1.0, 0.0);
        }
        else
        {
            b2_des = b2_des / n2;
        }

        Eigen::Vector3d b1_des = b2_des.cross(b3_des);

        Eigen::Matrix3d R_des;
        R_des.col(0) = b1_des;
        R_des.col(1) = b2_des;
        R_des.col(2) = b3_des;

        // Thrust limits
        T_des = clamp(T_des, p_.thrust_min, p_.thrust_max);

        *R_des_out = R_des;
        *T_des_out = T_des;
    }

    CascadeOutput CascadeController::update(const RigidBodyState &s,
                                            const Reference &r,
                                            double dt)
    {
        CascadeOutput out;

        // ---position loop : p -> v_des ---
        const Eigen::Vector3d e_p = r.p_des - s.p_n;
        Eigen::Vector3d v_corr = pos_pid_.step3(e_p, dt); // correction
        Eigen::Vector3d v_des = r.v_ff + v_corr;
        v_des = clampNorm(v_des, p_.v_max);

        // --- velocity loop: v->a_des ---
        const Eigen::Vector3d e_v = v_des - s.v_n;
        Eigen::Vector3d a_corr = vel_pid_.step3(e_v, dt);
        Eigen::Vector3d a_des = r.a_ff + a_corr;
        a_des = clampNorm(a_des, p_.a_max);

        // ---a_des -> (R_des, T_des) ---
        Eigen::Matrix3d R_des;
        double T_des = 0.0;
        accelToAttitudeThrust(a_des, r.yaw_des, &R_des, &T_des);

        // ---Attitude loop : R_des -> omega_des
        const Eigen::Matrix3d R_bn = quatToRbn(s.q_bn);
        const Eigen::Vector3d e_R = attitudeError(R_bn, R_des);

        Eigen::Vector3d omega_des = p_.kp_att.cwiseProduct(e_R);
        omega_des = clampNorm(omega_des, p_.omega_max);

        // Rate loop: omega -> tau_cmd ---
        const Eigen::Vector3d e_om = omega_des - s.omega_b;
        Eigen::Vector3d tau_cmd = rate_pid_.step3(e_om, dt);

        // Torque limits (per-axis)
        tau_cmd.x() = clamp(tau_cmd.x(), -p_.tau_max.x(), p_.tau_max.x());
        tau_cmd.y() = clamp(tau_cmd.y(), -p_.tau_max.y(), p_.tau_max.y());
        tau_cmd.z() = clamp(tau_cmd.z(), -p_.tau_max.z(), p_.tau_max.z());

        // Fill output
        out.v_des = v_des;
        out.a_des = a_des;
        out.R_des = R_des;
        out.omega_des = omega_des;

        out.thrust_cmd = T_des;
        out.tau_cmd = tau_cmd;
        out.wrench_cmd << T_des, tau_cmd.x(), tau_cmd.y(), tau_cmd.z();
        return out;
    }
}