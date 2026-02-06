#pragma onece
#include <Eigen/Dense>

namespace gnc
{

    // hat : R^3 -> so(3), w^ -> skew matrix so that w^v = w X v
    Eigen::Matrix3d hat(const Eigen::Vector3d &w);

    // vee: so(3) -> R^3, inverse of hat for skew matrices
    Eigen::Vector3d vee(const Eigen::Vector3d &W);

    // SO(3) attitude error:
    // e_R = 0.5 * vee(Rd^T R - R^T Rd)
    // Rd = R_d^n ; R=R_b^n
    Eigen::Vector3d so3Error(const Eigen::Matrix3d &R,
                             const Eigen::Matrix3d &Rd);

    // Common trace_based attitude error function (for plotting/metrics)
    // Psi(R,Rd) = 0.5*(3- tr(Rd^TR))
    double so3Psi(const Eigen::Matrix3d &R,
                  const Eigen::Matrix3d &Rd);

}