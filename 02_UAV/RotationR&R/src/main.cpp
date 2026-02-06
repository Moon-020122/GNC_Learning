#include <iostream>
#include <random>
#include <cmath>

#include "rotation.hpp"
#include "validation.hpp"

int main()
{
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> uni(-M_PI, M_PI);

    const double pitch_limit = deg2rad(89.0);
    std::uniform_real_distribution<double> pitch_dist(-pitch_limit, pitch_limit);

    const int N = 5000;

    double max_ortho = 0.0;
    double max_det = 0.0;
    double max_qnorm_err = 0.0;
    double max_Rdiff = 0.0;
    // Generate a set of roll/pitch/yaw randomly each time
    for (int i = 0; i < N; ++i)
    {
        const double roll = uni(rng);
        const double pitch = pitch_dist(rng);
        const double yaw = uni(rng);
        // matrix A ,rotation martix
        Eigen::Matrix3d R_e = gnc::eulerZYXToRbn(roll, pitch, yaw);
        Eigen::Quaterniond q = gnc::euluerZYXToQuat(roll, pitch, yaw);
        // (simulate "drift" then normalize, to show the check has meaning)
        Eigen::Quaterniond q_drift = q;
        q_drift.w() *= 1.000001; // tiny drift
        // If a disturbance is added, it will fail the check.
        //  Eigen::Quaterniond q_norm = gnc::normalizeQuat(q_drift);
        Eigen::Quaterniond q_norm = gnc::normalizeQuat(q);

        const double qnorm_err = std::abs(q_norm.norm() - 1.0);
        // Converting back from quaternions, that is B
        /*If the 90-degree limit of the pitch is not imposed here, then theoretically,
        the converted B is not necessarily equal to A. However, in engineering practice,
        only one-way operation is performed, so we need to impose this limit during testing.*/
        Eigen::Matrix3d R_q = gnc::quatToRbn(q_norm);

        // checks
        const double ortho_err = gnc::orthogonalityError(R_q);
        const double det_err = gnc::detError(R_q);
        const double Rdiff = gnc::rotationDiff(R_e, R_q);

        max_ortho = std::max(max_ortho, ortho_err);
        max_det = std::max(max_det, det_err);
        max_qnorm_err = std::max(max_qnorm_err, qnorm_err);
        max_Rdiff = std::max(max_Rdiff, Rdiff);
    }

    std::cout << "===== Validation Results (N=" << N << ") =====\n";
    std::cout << "Max ||R^T R - I||_inf      = " << max_ortho << "\n";
    std::cout << "Max |det(R) - 1|           = " << max_det << "\n";
    std::cout << "Max | ||q|| - 1 |          = " << max_qnorm_err << "\n";
    std::cout << "Max ||R_euler - R_quat||_inf= " << max_Rdiff << "\n";

    const double TH_ORTHO = 1e-10;
    const double TH_DET = 1e-10;
    const double TH_QNORM = 1e-12;
    const double TH_RDIFF = 1e-10;

    bool pass = true;
    pass &= (max_ortho < TH_ORTHO);
    pass &= (max_det < TH_DET);
    pass &= (max_qnorm_err < TH_QNORM);
    pass &= (max_Rdiff < TH_RDIFF);

    std::cout << "PASS? " << (pass ? "YES" : "NO") << "\n";

    bool pass_2 = true;
    pass_2 &= gnc::runModule2HoverTest();
    pass_2 &= gnc::runModule2ConstantOmegaTest();
    pass_2 &= gnc::runEulerOmegaMappingSanity();

    // module3
    bool pass_3;
    pass_3 = gnc::runModule3StepSizeStability();

    // module 4
    pass &= gnc::testPureThrust();
    pass &= gnc::testPureRollTorque();
    pass &= gnc::testSaturationSafety();
    pass &= gnc::testMotorLag(); // optional

    // cascade Pid module
    gnc::KinematicsParams plant;
    plant.mass = 1.5;
    plant.gravity = 9.81;
    plant.linear_drag = 0.2;

    // inertia(example)
    plant.rb.I_b = (Eigen::Vector3d(0.02, 0.03, 0.04)).asDiagonal();
    plant.rb.I_b_inv = plant.rb.I_b.inverse();
    plant.rb.rotor_J = 0.0;
    plant.rb.omega_spin_sum = 0.0;

    // --- Mixer & motors ---
    auto cfg = gnc::makeQuadX(0.25, 1e-5, 1e-6, 0.0, 2000.0);
    auto mx = gnc::buildMixer(cfg); // A Matrix

    gnc::MotorParams mp;
    mp.tau_m = 0.05;

    //---Controller ---
    gnc::CascadeController ctrl;
    auto cparams = gnc::makeDefaultControllerParams(plant.mass, plant.gravity);
    // 4* Maximum lift of a single rotor
    cparams.thrust_max = 4.0 * cfg.kT * cfg.omega_max * cfg.omega_max;

    ctrl.setParams(cparams);
    ctrl.reset();

    const double dt = 0.002;
    // Initial state
    gnc::RigidBodyState s0;
    s0.p_n = Eigen::Vector3d(0.0, 0.0, 0.0);
    s0.v_n = Eigen::Vector3d(0.0, 0.0, 0.0);
    s0.q_bn = Eigen::Quaterniond::Identity();
    s0.omega_b.setZero();

    // =====1) Hovering anti-disturbance =====
    gnc::simulateScenario(
        "Hover hold with disturbance",
        s0,
        plant, ctrl, cfg, mx, mp,
        dt, 8.0,
        [](double /*t*/)
        {
            gnc::Reference r;
            r.p_des = Eigen::Vector3d(0.0, 0.0, 0.0);
            r.v_ff.setZero();
            r.a_ff.setZero();
            r.yaw_des = 0.0;
            return r;
        },
        [](double t) -> Eigen::Vector3d
        {
            // Apply a northward acceleration disturbance for 0-1 seconds, then remove it.
            if (t < 1.0)
                return Eigen::Vector3d(0.8, 0.0, 0.0);
            return Eigen::Vector3d::Zero();
        });

    ctrl.reset();

    //=====2) Cruise control and braking to a stop =====
    gnc::simulateScenario(
        "cruise 1m/s then stop",
        s0, plant, ctrl, cfg, mx, mp,
        dt, 10.0,
        [](double t)
        {
            gnc::Reference r;
            // 1 m/s in the N direction, stops after five seconds
            const double v = (t < 5.0) ? 1.0 : 0.0;
            // Generate a consistent p_des: integrating to obtain the expected position to avoid conflicts
            // with the position loop and velocity feedforward.
            const double p = (t < 5.0) ? (1.0 * t) : (1.0 * 5.0);
            r.p_des = Eigen::Vector3d(p, 0.0, 0.0);
            r.v_ff = Eigen::Vector3d(v, 0.0, 0.0);
            r.a_ff = Eigen::Vector3d::Zero();
            r.yaw_des = 0.0;
            return r;
        },
        [](double /*t*/)
        { return Eigen::Vector3d::Zero(); });

    ctrl.reset();

    //=====3)circle tracking ====
    gnc::simulateScenario(
        "circle tracking",
        s0,
        plant, ctrl, cfg, mx, mp,
        dt, 12.0,
        [](double t)
        {
            gnc::Reference r;
            const double R = 2.0;              // meters
            const double w = 2.0 * M_PI / 8.0; // rad/s (period 8s)  angle velocity
            const double ct = std::cos(w * t);
            const double st = std::sin(w * t);

            r.p_des = Eigen::Vector3d(R * ct, R * st, 0.0);
            r.v_ff = Eigen::Vector3d(-R * w * st, R * w * ct, 0.0);
            r.a_ff = Eigen::Vector3d(-R * w * w * ct, -R * w * w * st, 0.0);
            r.yaw_des = 0.0;
            return r;
        },
        [](double /*t*/)
        { return Eigen::Vector3d::Zero(); });

    //  geometry controller
    gnc::KinematicsParams plant;
    plant.mass = 1.5;
    plant.gravity = 9.81;
    plant.linear_drag = 0.0; // no drag

    plant.rb.I_b = (Eigen::Vector3d(0.02, 0.03, 0.04)).asDiagonal();
    plant.rb.I_b_inv = plant.rb.I_b.inverse();
    plant.rb.rotor_J = 0.0;
    plant.rb.omega_spin_sum = 0.0;

    // ----- Initial state: large maneuver ----
    gnc::RigidBodyState s0;
    s0.p_n.setZero();
    s0.v_n.setZero();
    // bit tilt
    const double roll = deg2rad(170.0);
    const double pitch = deg2rad(60.0);
    const double yaw = deg2rad(20.0);
    s0.q_bn = gnc::euluerZYXToQuat(roll, pitch, yaw);
    s0.omega_b.setZero();

    // Desired attitude: level
    Eigen::Matrix3d Rd = Eigen::Matrix3d::Identity();

    // ---- Geometric controller ----
    gnc::GeomAttParams gp;
    gp.I = plant.rb.I_b;
    gp.I_inv = plant.rb.I_b_inv;
    gp.kR = 10.0;
    gp.kW = 2.0;
    gp.tau_max = Eigen::Vector3d(1.2, 1.2, 0.6);

    gnc::GeomAttController gc;
    gc.setParams(gp);

    const double dt = 0.001;
    const double Tsim = 6.0;

    auto m = runGeomAttitudeFlipTest(
        "geom_att_flip.csv",
        s0, plant, gc,
        dt, Tsim, Rd);

    std::cout << "===== Geom Attitude Flip Metrics =====\n";
    std::cout << "max ||eR||   = " << m.max_eR << "\n";
    std::cout << "max Psi      = " << m.max_psi << "\n";
    std::cout << "max ||omega||= " << m.max_omega << "\n";
    std::cout << "max orthoErr = " << m.max_ortho << "\n";
    std::cout << "max detErr   = " << m.max_det << "\n";
    std::cout << "settle time  = " << m.settle_time << " s\n";
    std::cout << "CSV written: geom_att_flip.csv\n";

    return pass
               ? 0
               : 1;
}