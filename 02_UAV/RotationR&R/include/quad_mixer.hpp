#pragma once
#include <Eigen/Dense>
#include <array>

namespace gnc
{
    struct QuadConfig
    {
        double arm = 0.25; // meters (motor distance from center projected)
        // Four three-dimensional vectors, representing positions relative to the airframe
        std::array<Eigen::Vector3d, 4> r_b; // motor positions in body (x forward, y right)

        // Aerodynamic cofficients
        double kT = 1e-5; // N/(rad/s)^2;
        double kQ = 1e-6; // N*m(rad/s)^2;

        // spin direction signs for yaw torque (+1 or -1)
        // Choose a convention and keep consistent
        //  s_i
        std::array<int, 4> spin;

        // Limits
        double omega_min = 0.0;    // rad/s
        double omega_max = 2000.0; // rad/s
    };
    // X type quad drone
    QuadConfig makeQuadX(
        double arm,
        double kT, double kQ,
        double omega_min, double omega_max);

    struct Mixer
    {
        Eigen::Matrix4d A = Eigen::Matrix4d::Zero();     // u =  A*w^2
        Eigen::Matrix4d A_inv = Eigen::Matrix4d::Zero(); // w^2 = A_inv *u (if invertible)
    };
    // build allocation matrix from config
    Mixer buildMixer(const QuadConfig &cfg);

    // forward : omega->u  
    Eigen::Vector4d wrenchFromOmega(const QuadConfig &cfg, const Eigen::Vector4d &omega);

    // Inverse allocation : u -> omega_cmd with saturation, also outpus achieved wrench
    Eigen::Vector4d allocateOmegaCmd(const QuadConfig &cfg, const Mixer &mx,
                                      const Eigen::Vector4d &u_cmd, Eigen::Vector4d *u_ach_out);
}