#include "kinematics.hpp"
#include "rotation.hpp"
#include "dynamics.hpp"

namespace gnc
{

    void stepKinematics(
        RigidBodyState &s,
        const ControlInput &u,
        const KinematicsParams &params,
        const double dt)
    {
        // 1) omega_dot from Euler equation
        Eigen::Vector3d omega_dot = omegaDotEuler(s.omega_b, u.tau_b, params.rb);
        s.omega_b += omega_dot *dt;

        // solving th quaternion differential equation and then integrating it
        /*Here, s.omega_b should be passed instead of u.omega_b; the aircraft's own attitude 
            should be used rather than artificially providing the angular velocity.*/
        s.q_bn = integrateQuatBodyRate(s.q_bn, s.omega_b, dt);
        // revolve matrix, R_b^n from quaternion
        const Eigen::Matrix3d R_bn = quatToRbn(s.q_bn);

        const Eigen::Vector3d g_n(0.0, 0.0, params.gravity); // grivity
        const Eigen::Vector3d f_b(0.0, 0.0, -u.thrust);      // thrust
        // Acceleration in NED
        Eigen::Vector3d a_n = g_n + (1.0 / params.mass) * (R_bn * f_b);

        // a_drag = -k*v
        a_n += -params.linear_drag * s.v_n;

        // Semi-implicit Euler: update v then p (more stable than pure Euler)
        s.v_n += a_n * dt;
        s.p_n += s.v_n * dt;
    }
}