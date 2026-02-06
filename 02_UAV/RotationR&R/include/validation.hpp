#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "cascade_controller.hpp"
#include "quad_mixer.hpp"
#include "motor_model.hpp"
#include "geom_att_controller.hpp"

namespace gnc
{
    // validation merics
    double orthogonalityError(const Eigen::Matrix3d &R);                     //||R^T*R - I||_inf
    double detError(const Eigen::Matrix3d &R);                               // |det(R) - 1|
    double rotationDiff(const Eigen::Matrix3d &A, const Eigen::Matrix3d &B); //||A-B||_inf

    bool runModule2HoverTest();
    bool runModule2ConstantOmegaTest();
    bool runEulerOmegaMappingSanity();

    // module 3
    bool runModule3SingleAxisTorque(double dt);
    bool runModule3StepSizeStability();

    // module 4
    bool testPureThrust();
    bool testPureRollTorque();
    bool testSaturationSafety();
    bool testMotorLag();

    // Cascade Pid
    gnc::CascadeParams makeDefaultControllerParams(double mass, double g);
    void simulateScenario(
        const char *name,
        gnc::RigidBodyState s,
        gnc::KinematicsParams plant_params,
        gnc::CascadeController &ctrl,
        const gnc::QuadConfig &cfg,
        const gnc::Mixer &mx,
        gnc::MotorParams mp,
        double dt, double Tsim,
        std::function<gnc::Reference(double)> ref_fn,
        std::function<Eigen::Vector3d(double)> disturbance_acc_n_fn);

    // geometery attitude controller module
    struct AttMetrics
    {
        double max_eR = 0.0;
        double max_psi = 0.0;
        double max_omega = 0.0;
        double max_ortho = 0.0;
        double max_det = 0.0;
        double settle_time = -1.0;
    };
    static AttMetrics runGeomAttitudeFlipTest(
        const char *csv_path,
        gnc::RigidBodyState s,
        gnc::KinematicsParams plant,
        const gnc::GeomAttController &gc,
        double dt, double Tsim,
        const Eigen::Matrix3d &Rd);
}