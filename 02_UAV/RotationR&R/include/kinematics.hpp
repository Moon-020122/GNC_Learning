#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "dynamics.hpp"

namespace gnc
{

    struct RigidBodyState
    {
        Eigen::Vector3d p_n = Eigen::Vector3d::Zero(); //position
        Eigen::Vector3d v_n = Eigen::Vector3d::Zero(); //velocity
        Eigen::Quaterniond q_bn = Eigen::Quaterniond::Identity(); //quaternion

        Eigen::Vector3d omega_b = Eigen::Vector3d::Zero(); //body angular velocity
    };
    struct KinematicsParams
    {
        double mass = 1.0;
        double gravity = 9.81;
        double linear_drag = 0.0; // 1/s, simple model: a_drag = -linear_drag * v

        RigidBodyParams rb;
    };

    struct ControlInput
    {
        double thrust = 0.0;
        Eigen::Vector3d omega_b = Eigen::Vector3d::Zero();  
        Eigen::Vector3d tau_b = Eigen::Vector3d::Zero();
    };

    void stepKinematics(
        RigidBodyState &s,
        const ControlInput &u,
        const KinematicsParams &param,
        double dt);
};