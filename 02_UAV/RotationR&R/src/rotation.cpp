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
    double orthogonalityError(const Eigen::Matrix3d &R)
    {
        Eigen::Matrix3d E = R.transpose() * R - Eigen::Matrix3d::Identity();
        return E.cwiseAbs().maxCoeff(); // infinity norm (max abs element)
    }
    double detError(const Eigen::Matrix3d &R)
    {
        return std::abs(R.determinant() - 1.0);
    }
    double rotationDiff(const Eigen::Matrix3d &A, const Eigen::Matrix3d &B)
    {
        return (A - B).cwiseAbs().maxCoeff();
    }

}