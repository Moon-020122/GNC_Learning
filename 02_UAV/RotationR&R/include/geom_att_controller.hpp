#pragma once
#include <Eigen/Dense>

namespace gnc
{
    struct GeomAttParams
    {
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d I_inv = Eigen::Matrix3d::Identity();

        double kR = 8.0;
        double kW = 1.5;

        Eigen::Vector3d tau_max = Eigen::Vector3d(1.2, 1.2, 0.6); // N*m torque
    };

    struct GeomAttOutput
    {
        Eigen::Vector3d eR = Eigen::Vector3d::Zero();
        Eigen::Vector3d eW = Eigen::Vector3d::Zero();
        double psi = 0.0; //  trace_error
        Eigen::Vector3d tau = Eigen::Vector3d::Zero();
    };

    class GeomAttController
    {
    public:
        void setParams(const GeomAttParams &p) { p_ = p; }
        const GeomAttParams &params() const { return p_; }

        // R, Rd are body->NED rotation matrices
        // omega, omega_d are body rates (expressed in body frame)
        // omega_d_dot can be zero for basic tracking

        GeomAttOutput update(const Eigen::Matrix3d &R,
                             const Eigen::Vector3d &omega,
                             const Eigen::Matrix3d &Rd,
                             const Eigen::Vector3d &omega_d,
                             const Eigen::Vector3d &omega_d_dot) const;

    private:
        GeomAttParams p_;

        static double clamp(double x, double lo, double hi);
    };
}