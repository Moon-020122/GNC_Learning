#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>

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

    // validation merics
    double orthogonalityError(const Eigen::Matrix3d &R);  //||R^T*R - I||_inf
    double detError(const Eigen::Matrix3d& R); // |det(R) - 1|
    double rotationDiff(const Eigen::Matrix3d& A, const Eigen::Matrix3d& B); //||A-B||_inf
}