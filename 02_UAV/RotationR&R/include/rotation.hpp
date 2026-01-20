#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>


static double deg2rad(double deg) { return deg * M_PI / 180.0; }

namespace gnc
{
    // Euler angles are roll(phi), pitch(theta), yaw(psi), with ZYX order:
    // R_b^n = Rz(yaw) * Ry(pitch) * Rx(roll)
    // Transform: v^n = R_b^n * v^b
    Eigen::Matrix3d eulerZYXToRbn(double roll, double pitch, double yaw);

    // Convert Euler ZYX (roll,pitch,yaw) to quaternion q (w,x,y,z)
    // Consistent with R_b^n = Rz * Ry * Rx
    Eigen::Quaterniond euluerZYXToQuat(double roll, double pitch, double yaw);

    // convert quaternion to rotation matrix R_b^n (manual formula)
    Eigen::Matrix3d quatToRbn(const Eigen::Quaterniond &q);

    // Normalize quaternion
    Eigen::Quaterniond normalizeQuat(const Eigen::Quaterniond &q);

    Eigen::Quaterniond integrateQuatBodyRate(
        const Eigen::Quaterniond &q_bn,
        const Eigen::Vector3d &omega_b,
        double dt);

    Eigen::Vector3d eulerRatesToBodyOmegaZYX(
        double roll, double pitch,
        const Eigen::Vector3d &euler_dot);

    bool bodyOmegaToEulerRatesZYX(
        double roll, double pitch,
        const Eigen::Vector3d &omega_b,
        Eigen::Vector3d *euler_dot_out);


}