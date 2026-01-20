#include "rotation.hpp"
#include <cmath>
#include <limits>

namespace gnc
{
    Eigen::Matrix3d eulerZYXToRbn(double roll, double pitch, double yaw)
    {
        const double cr = std::cos(roll);
        const double sr = std::sin(roll);
        const double cp = std::cos(pitch);
        const double sp = std::sin(pitch);
        const double cy = std::cos(yaw);
        const double sy = std::sin(yaw);

        // R = Rz(yaw) * Ry(pitch) * Rx(roll)
        Eigen::Matrix3d R;
        R << cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
            sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
            -sp, cp * sr, cp * cr;
        return R;
    }

    Eigen::Quaterniond euluerZYXToQuat(double roll, double pitch, double yaw)
    {
        // ZYX: q = qz(yaw) ⊗ qy(pitch) ⊗ qx(roll)
        const double half_r = roll * 0.5;
        const double half_p = pitch * 0.5;
        const double half_y = yaw * 0.5;

        const double cr = std::cos(half_r);
        const double sr = std::sin(half_r);
        const double cp = std::cos(half_p);
        const double sp = std::sin(half_p);
        const double cy = std::cos(half_y);
        const double sy = std::sin(half_y);
        // ZYX: q = qz(yaw) ⊗ qy(pitch) ⊗ qx(roll) Expanding it gives the following formula

        const double w = cy * cp * cr + sy * sp * sr;
        const double x = cy * cp * sr - sy * sp * cr;
        const double y = sy * cp * sr + cy * sp * cr;
        const double z = sy * cp * cr - cy * sp * sr;

        Eigen::Quaterniond q(w, x, y, z);
        return normalizeQuat(q);
    }
    Eigen::Matrix3d quatToRbn(const Eigen::Quaterniond &q_in)
    {
        Eigen::Quaterniond q = normalizeQuat(q_in);
        const double w = q.w();
        const double x = q.x();
        const double y = q.y();
        const double z = q.z();

        Eigen::Matrix3d R;
        R << 1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y),
            2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x),
            2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y);
        return R;
    }
    Eigen::Quaterniond normalizeQuat(const Eigen::Quaterniond &q)
    {
        const double n = q.norm();
        if (n < 1e-12 || !std::isfinite(n))
        {
            return Eigen::Quaterniond::Identity();
        }
        Eigen::Quaterniond out = q;
        out.normalize();
        return out;
    }


    Eigen::Quaterniond integrateQuatBodyRate(
        const Eigen::Quaterniond &q_bn,
        const Eigen::Vector3d &omega_b,
        double dt)
    {
        const double p = omega_b.x();
        const double q = omega_b.y();
        const double r = omega_b.z();

        const double w = q_bn.w();
        const double x = q_bn.x();
        const double y = q_bn.y();
        const double z = q_bn.z();

        // q_dot = 0.5 * Omega(omega) * q (body rates)

        Eigen::Quaterniond q_dot;
        q_dot.w() = -0.5 * (x * p + y * q + z * r);
        q_dot.x() = 0.5 * (w * p + y * r - z * q);
        q_dot.y() = 0.5 * (w * q + z * p - x * r);
        q_dot.z() = 0.5 * (w * r + x * q - y * p);

        Eigen::Quaterniond q_next(
            w + q_dot.w() * dt,
            x + q_dot.x() * dt,
            y + q_dot.y() * dt,
            z + q_dot.z() * dt);

        return normalizeQuat(q_next);
    }

    Eigen::Vector3d eulerRatesToBodyOmegaZYX(
        double roll, double pitch,
        const Eigen::Vector3d &euler_dot)
    {
        const double phi = roll;
        const double theta = pitch;

        const double cphi = std::cos(phi);
        const double sphi = std::sin(phi);
        const double cth = std::cos(theta);
        const double sth = std::sin(theta);

        Eigen::Matrix3d E;
        E << 1.0, 0.0, -sth,
            0.0, cphi, sphi * cth,
            0.0, -sphi, cphi * cth;

        return E * euler_dot;
    }

    bool bodyOmegaToEulerRatesZYX(
        double roll, double pitch,
        const Eigen::Vector3d &omega_b,
        Eigen::Vector3d *euler_dot_out)
    {
        const double phi = roll;
        const double theta = pitch;

        const double cphi = std::cos(phi);
        const double sphi = std::sin(phi);
        const double cth = std::cos(theta);

        // singular when cos(theta) -> 0
        if (std::abs(cth) < 1e-6)
        {
            return false;
        }

        const double tth = std::tan(theta);

        Eigen::Matrix3d W;
        W << 1.0, sphi * tth, cphi * tth,
            0.0, cphi, -sphi,
            0.0, sphi / cth, cphi / cth;

        *euler_dot_out = W * omega_b;
        return true;
    }

}