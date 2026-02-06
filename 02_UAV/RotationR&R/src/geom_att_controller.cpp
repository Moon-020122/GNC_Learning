#include "geom_att_controller.hpp"
#include "lie.hpp"
#include <algorithm>

namespace gnc
{
    double GeomAttController::clamp(double x, double lo, double hi)
    {
        return std::min(std::max(x, lo), hi);
    }

    GeomAttOutput GeomAttController::update(const Eigen::Matrix3d &R,
                                            const Eigen::Vector3d &omega,
                                            const Eigen::Matrix3d &Rd,
                                            const Eigen::Vector3d &omega_d,
                                            const Eigen::Vector3d &omega_d_dot) const
    {
        GeomAttOutput out;

        out.eR = gnc::so3Error(R, Rd);
        out.psi = gnc::so3Psi(R, Rd);

        // eW = omega - R^T Rd omega_d
        const Eigen::Vector3d omega_d_in_body = R.transpose() * Rd * omega_d;
        out.eW = omega - omega_d_in_body;

        // Gyro coupling term : omega x(I omega)
        const Eigen::Vector3d Iw = p_.I * omega;
        const Eigen::Vector3d gyro = omega.cross(Iw);

        // Feedforward term (0 if omega_d, omega_d_dot are zero)
        const Eigen::Vector3d ff = -p_.I * (hat(omega) * omega_d_in_body - R.transpose() * Rd * omega_d_dot);

        Eigen::Vector3d tau = -p_.kR * out.eR - p_.kW * out.eW + gyro + ff;

        // Saturation
        tau.x() = clamp(tau.x(), -p_.tau_max.x(), p_.tau_max.x());
        tau.y() = clamp(tau.y(), -p_.tau_max.y(), p_.tau_max.y());
        tau.z() = clamp(tau.z(), -p_.tau_max.z(), p_.tau_max.z());

        out.tau = tau;
        return out;
    }

}