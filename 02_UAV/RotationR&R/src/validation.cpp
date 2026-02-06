#include "validation.hpp"
#include <cmath>
#include <limits>
#include "rotation.hpp"
#include <iostream>
#include "kinematics.hpp"
// module 4
#include "motor_model.hpp"
#include "quad_mixer.hpp"
// geometery controller
#include <fstream>

namespace gnc
{
    // model 1
    double orthogonalityError(const Eigen::Matrix3d &R)
    {
        Eigen::Matrix3d E = R.transpose() * R - Eigen::Matrix3d::Identity();
        return E.cwiseAbs().maxCoeff(); // infinity norm (max abs element)
    }
    double detError(const Eigen::Matrix3d &R)
    {
        return std::abs(R.determinant() - 1.0);
    }
    double rotationDiff(const Eigen::Matrix3d &A, const Eigen::Matrix3d &B)
    {
        return (A - B).cwiseAbs().maxCoeff();
    }
    // model 2
    bool runModule2HoverTest()
    {
        std::cout << "\n =====module 2 Test A: Level Hover ======\n";

        gnc::KinematicsParams params;
        params.mass = 1.5;
        params.gravity = 9.81;
        params.linear_drag = 0.8;

        gnc::RigidBodyState s;
        s.p_n = Eigen::Vector3d(0, 0, 0);
        s.v_n = Eigen::Vector3d(1.0, -0.5, 0.2); // non-zero initial velocity to test convergence
        s.q_bn = Eigen::Quaterniond::Identity();

        gnc::ControlInput u;
        u.thrust = params.mass * params.gravity; // T= mg ,hover
        u.omega_b.setZero();                     // no rotation

        const double dt = 0.01;
        const double Tsim = 8.0;
        const int N = static_cast<int>(Tsim / dt);

        for (int i = 0; i < N; ++i)
        {
            gnc::stepKinematics(s, u, params, dt);
        }

        // Checks
        // 1) attitude unchanged
        // no revolve if quat_dot = 1, Note the definition of quaternions 注意四元数定义
        const double quat_dot = std::abs(s.q_bn.dot(Eigen::Quaterniond::Identity())); // Identity = (1,0,0,0)
        const double attitude_ok = (1.0 - quat_dot) < 1e-10;                          // dot close to 1 means same orientation

        // 2)velocity converged (because drag enabled)
        const double v_norm = s.v_n.norm(); // modulo

        // 3） thrust approx mg
        const double thrust_err = std::abs(u.thrust - params.mass * params.gravity);

        std::cout << "Final |v| = " << v_norm << "m/s \n";
        std::cout << "Attitude dot  (identity) = " << quat_dot << "\n";
        std::cout << "|T-mg| = " << thrust_err << "N \n";

        const bool pass = attitude_ok && (v_norm < 0.02) && (thrust_err < 1e-12);
        std::cout << "Pass?" << (pass ? "yes" : "no") << "\n";

        return pass;
    }

    bool runModule2ConstantOmegaTest()
    {
        std::cout << "\n ====Module 2 TestB: Constant Body Rate =====\n";

        gnc::RigidBodyState s;
        s.q_bn = Eigen::Quaterniond::Identity();

        gnc::KinematicsParams params;
        params.mass = 1.0;
        params.gravity = 9.81;
        params.linear_drag = 0.0;

        gnc::ControlInput u;
        u.thrust = params.mass * params.gravity;
        const double yaw_rate = deg2rad(30.0); // 30deg/s
        u.omega_b = Eigen::Vector3d(0, 0, yaw_rate);

        const double dt = 0.001;
        const double Tsim = 2.0;
        const int N = static_cast<int>(Tsim / dt);

        for (int i = 0; i < N; ++i)
        {
            // only attitude matters here; position/velocity are unused but harmless
            // Here, s is &，and not const, which will change the value of q_bn.
            gnc::stepKinematics(s, u, params, dt);
        }

        const double yaw_expected = yaw_rate * Tsim;
        Eigen::Matrix3d R_expected = gnc::eulerZYXToRbn(0.0, 0.0, yaw_expected);
        Eigen::Matrix3d R_sim = gnc::quatToRbn(s.q_bn);

        const double Rdiff = gnc::rotationDiff(R_expected, R_sim);
        std::cout << "Expected yaw = " << yaw_expected << " rad\n";
        std::cout << "Max ||R_expected - R_sim||_inf = " << Rdiff << "\n";

        const bool pass = (Rdiff < 5e-4); // tolerance accounts for Euler integration error
        std::cout << "PASS? " << (pass ? "YES" : "NO") << "\n";
        return pass;
    }

    bool runEulerOmegaMappingSanity()
    {
        std::cout << "\n ===== Module 2 Bonus ： Euler rate <-> omega mapping sanity===== \n";
        const double roll = deg2rad(20.0);
        const double pitch = deg2rad(30.0);
        Eigen::Vector3d euler_dot;
        euler_dot << deg2rad(10.0), deg2rad(-5.0), deg2rad(15.0);

        Eigen::Vector3d omega_b = gnc::eulerRatesToBodyOmegaZYX(roll, pitch, euler_dot);

        Eigen::Vector3d euler_dot_rec;
        const bool ok = gnc::bodyOmegaToEulerRatesZYX(roll, pitch, omega_b, &euler_dot_rec);

        std::cout << "omega_b = [" << omega_b.transpose() << "]\n";
        if (!ok)
        {
            std::cout << "FAIL: near singular (pitch ~ +-90deg)\n";
            return false;
        }

        const double err = (euler_dot - euler_dot_rec).cwiseAbs().maxCoeff();
        std::cout << "Max |euler_dot - recovered| = " << err << "\n";

        const bool pass = err < 1e-10;
        std::cout << "PASS? " << (pass ? "YES" : "NO") << "\n";
        return pass;
    }

    // module 3

    bool runModule3SingleAxisTorque(double dt)
    {
        std::cout << "\n ====== module 3 Test 1:Single Axis Torque (dt = " << dt << ") =====\n";

        gnc::KinematicsParams params;
        params.mass = 1.0;
        params.gravity = 9.81;
        params.linear_drag = 0.0;

        // Diagonal inertia for clean expectation
        const double Ix = 0.02, Iy = 0.03, Iz = 0.04;
        params.rb.I_b = (Eigen::Vector3d(Ix, Iy, Iz)).asDiagonal(); // moment of inertia
        params.rb.I_b_inv = params.rb.I_b.inverse();

        // disable rotor gyro
        params.rb.rotor_J = 0.0;
        params.rb.omega_spin_sum = 0.0;

        gnc::RigidBodyState s;
        s.q_bn = Eigen::Quaterniond::Identity();
        s.omega_b.setZero();

        gnc::ControlInput u;
        u.thrust = params.mass * params.gravity;
        u.tau_b = Eigen::Vector3d(0.01, 0.0, 0.0); // external torque

        const double Tsim = 2.0;
        const int N = static_cast<int>(Tsim / dt);

        for (int i = 0; i < N; ++i)
        {
            gnc::stepKinematics(s, u, params, dt);
        }

        // Expected: omega_x = (tau_x/Ix)*T  Newton's second law
        const double omega_x_expected = (u.tau_b.x() / Ix) * Tsim;
        // s is calculated using Euler's formula and is simulated
        const double err = std::abs(s.omega_b.x() - omega_x_expected);

        std::cout << "omega_b final = [" << s.omega_b.transpose() << "]\n";
        std::cout << "expected omega_x = " << omega_x_expected << "\n";
        std::cout << "|omega_x - expected| = " << err << "\n";

        // For semi-implicit Euler, error should be small with small dt
        const bool pass = (err < 2e-3) && (std::abs(s.omega_b.y()) < 1e-3) && (std::abs(s.omega_b.z()) < 1e-3);
        std::cout << "PASS? " << (pass ? "YES" : "NO") << "\n";

        return pass;
    }

    bool runModule3StepSizeStability()
    {
        std::cout << "\n ===== Module 3 Test 2:Step Size Stability ======\n";
        bool pass1 = runModule3SingleAxisTorque(0.001);
        bool pass2 = runModule3SingleAxisTorque(0.005);
        bool pass = pass1 && pass2;
        std::cout << "PASS BOTH dts? " << (pass ? "YES" : "NO") << "\n";
        return pass;
    }

    // module 4
    //  pure thrust, Four motors increase simultaneously
    bool testPureThrust()
    {
        std::cout << "\n===== Module4 Test A: Pure Thrust =====\n";

        QuadConfig cfg = gnc::makeQuadX(0.25, 1e-5, 1e-6, 0.0, 2000.0);
        Mixer mx = gnc::buildMixer(cfg);

        Eigen::Vector4d u_cmd;
        u_cmd << 9.81, 0.0, 0.0, 0.0; // hover

        Eigen::Vector4d u_ach;
        Eigen::Vector4d omega_cmd = gnc::allocateOmegaCmd(cfg, mx, u_cmd, &u_ach);

        std::cout << "omega_cmd = " << omega_cmd.transpose() << "\n";
        std::cout << "u_ach     = " << u_ach.transpose() << "  [T, tx, ty, tz]\n";

        // check all equal (within tolerance)
        double maxdiff = 0.0;
        for (int i = 0; i < 4; ++i)
            maxdiff = std::max(maxdiff, std::abs(omega_cmd[i] - omega_cmd[0]));
        bool pass = (maxdiff < 1e-6);
        std::cout << "PASS? " << (pass ? "YES" : "NO") << "\n";
        return pass;
    }

    bool testPureRollTorque()
    {
        std::cout << "\n===== Module4 Test B: Pure Roll Torque =====\n";

        QuadConfig cfg = gnc::makeQuadX(0.25, 1e-5, 1e-6, 0.0, 2000.0);
        Mixer mx = gnc::buildMixer(cfg);

        Eigen::Vector4d u_cmd;
        u_cmd << 9.81, 0.2, 0.0, 0.0; // add + tau_x

        Eigen::Vector4d u_ach;
        Eigen::Vector4d omega_cmd = gnc::allocateOmegaCmd(cfg, mx, u_cmd, &u_ach);

        std::cout << "omega_cmd = " << omega_cmd.transpose() << "\n";
        std::cout << "u_ach     = " << u_ach.transpose() << "  [T, tx, ty, tz]\n";

        // With our numbering:
        // 1 front-left (y<0), 4 rear-left (y<0)  => contribute +tau_x when thrust increases? (because tau_x=-y*T)
        // 2 front-right (y>0),3 rear-right (y>0) => contribute -tau_x when thrust increases
        // So to make +tau_x, motors with y<0 tend to have larger thrust (larger omega)
        bool pass = (omega_cmd[0] > omega_cmd[1]) && (omega_cmd[3] > omega_cmd[2]);
        std::cout << "PASS? " << (pass ? "YES" : "NO") << "\n";
        return pass;
    }

    bool testSaturationSafety()
    {
        std::cout << "\n===== Module4 Test C: Saturation Safety =====\n";
        QuadConfig cfg = gnc::makeQuadX(0.25, 1e-5, 1e-6, 0.0, 2000.0);
        Mixer mx = gnc::buildMixer(cfg);

        Eigen::Vector4d u_cmd;
        u_cmd << 30.0, 5.0, 5.0, 2.0; // intentionally aggressive

        Eigen::Vector4d u_ach;
        Eigen::Vector4d omega_cmd = gnc::allocateOmegaCmd(cfg, mx, u_cmd, &u_ach);

        std::cout << "omega_cmd = " << omega_cmd.transpose() << "\n";
        std::cout << "u_ach     = " << u_ach.transpose() << "  [T, tx, ty, tz]\n";

        bool finite = omega_cmd.allFinite() && u_ach.allFinite();
        bool nonneg = (omega_cmd.minCoeff() >= 0.0);
        bool within = (omega_cmd.maxCoeff() <= cfg.omega_max + 1e-9);

        bool pass = finite && nonneg && within;
        std::cout << "PASS? " << (pass ? "YES" : "NO") << "\n";
        return pass;
    }

    bool testMotorLag()
    {
        std::cout << "\n===== Module4 Bonus: Motor Lag =====\n";

        auto cfg = gnc::makeQuadX(0.25, 1e-5, 1e-6, 0.0, 2000.0);
        auto mx = gnc::buildMixer(cfg);

        gnc::MotorParams mp;
        mp.tau_m = 0.05;
        gnc::MotorState ms;

        Eigen::Vector4d u_cmd;
        u_cmd << 9.81, 0.0, 0.0, 0.0;

        Eigen::Vector4d u_ach;
        Eigen::Vector4d omega_cmd = gnc::allocateOmegaCmd(cfg, mx, u_cmd, &u_ach);

        const double dt = 0.01;
        for (int k = 0; k < 200; ++k)
        {
            gnc::stepMotors(ms, mp, omega_cmd, dt);
        }

        std::cout << "omega_cmd = " << omega_cmd.transpose() << "\n";
        std::cout << "omega     = " << ms.omega.transpose() << " (should approach omega_cmd)\n";

        bool pass = ((ms.omega - omega_cmd).cwiseAbs().maxCoeff() < 1e-2);
        std::cout << "PASS? " << (pass ? "YES" : "NO") << "\n";
        return pass;
    }

    // cascade Pid module
    gnc::CascadeParams makeDefaultControllerParams(double mass, double g)
    {
        gnc::CascadeParams p;
        p.mass = mass;
        p.gravity = g;

        p.v_max = 3.0;
        p.a_max = 6.0;
        p.tilt_max_rad = deg2rad(35.0);
        p.omega_max = deg2rad(300.0);
        p.tau_max = Eigen::Vector3d(1.2, 1.2, 0.6);

        p.thrust_min = 0.0;
        p.thrust_max = 80.0;

        p.kp_att = Eigen::Vector3d(9.0, 9.0, 5.0);

        // ---position PID: p -> v_corr ---
        p.pos_pid_cfg.kp = Eigen::Vector3d(3.0, 3.0, 3.0);
        p.pos_pid_cfg.ki = Eigen::Vector3d(0.0, 0.0, 0.0);
        p.pos_pid_cfg.kd = Eigen::Vector3d(0.0, 0.0, 0.0);
        p.pos_pid_cfg.u_min = Eigen::Vector3d(-p.v_max, -p.v_max, -p.v_max);
        p.pos_pid_cfg.u_max = Eigen::Vector3d(+p.v_max, +p.v_max, +p.v_max);
        p.pos_pid_cfg.i_min = Eigen::Vector3d(-2.0, -2.0, -2.0);
        p.pos_pid_cfg.i_max = Eigen::Vector3d(+2.0, +2.0, +2.0);
        p.pos_pid_cfg.kaw = Eigen::Vector3d(0.0, 0.0, 0.0);
        p.pos_pid_cfg.d_filter_tau = 0.02;
        p.pos_pid_cfg.enable_integrator = false;
        p.pos_pid_cfg.enable_derivative = false;
        // ---velocity PID: v-> a_corr ---
        p.vel_pid_cfg.kp = Eigen::Vector3d(4.0, 4.0, 3.0);
        p.vel_pid_cfg.ki = Eigen::Vector3d(0.2, 0.2, 0.2);
        p.vel_pid_cfg.kd = Eigen::Vector3d(1.2, 1.2, 0.8);
        p.vel_pid_cfg.u_min = Eigen::Vector3d(-p.a_max, -p.a_max, -p.a_max);
        p.vel_pid_cfg.u_max = Eigen::Vector3d(+p.a_max, +p.a_max, +p.a_max);
        p.vel_pid_cfg.i_min = Eigen::Vector3d(-3.0, -3.0, -3.0);
        p.vel_pid_cfg.i_max = Eigen::Vector3d(+3.0, +3.0, +3.0);
        p.vel_pid_cfg.kaw = Eigen::Vector3d(0.2, 0.2, 0.2);
        p.vel_pid_cfg.d_filter_tau = 0.03;
        p.vel_pid_cfg.enable_integrator = true;
        p.vel_pid_cfg.enable_derivative = true;
        //---Rate PID: omega->tau ---
        p.rate_pid_cfg.kp = Eigen::Vector3d(0.60, 0.60, 0.30);
        p.rate_pid_cfg.ki = Eigen::Vector3d(0.1, 0.1, 0.05);
        p.rate_pid_cfg.kd = Eigen::Vector3d(0.02, 0.02, 0.02);
        p.rate_pid_cfg.u_min = Eigen::Vector3d(-p.tau_max.x(), -p.tau_max.y(), -p.tau_max.z());
        p.rate_pid_cfg.u_max = Eigen::Vector3d(+p.tau_max.x(), +p.tau_max.y(), +p.tau_max.z());
        p.rate_pid_cfg.i_min = Eigen::Vector3d(-2.0, -2.0, -2.0);
        p.rate_pid_cfg.i_max = Eigen::Vector3d(+2.0, +2.0, +2.0);
        p.rate_pid_cfg.kaw = Eigen::Vector3d(0.3, 0.3, 0.3); // anti-windup
        p.rate_pid_cfg.d_filter_tau = 0.02;
        p.rate_pid_cfg.enable_integrator = true;
        p.rate_pid_cfg.enable_derivative = true;

        return p;
    }

    void simulateScenario(const char *name,
                          gnc::RigidBodyState s,
                          gnc::KinematicsParams plant_params,
                          gnc::CascadeController &ctrl,
                          const gnc::QuadConfig &cfg,
                          const gnc::Mixer &mx,
                          gnc::MotorParams mp,
                          double dt, double Tsim,
                          std::function<gnc::Reference(double)> ref_fn,
                          std::function<Eigen::Vector3d(double)> disturbance_acc_n_fn)
    {
        std::cout << "\n===== Scenario: " << name << " =====\n";
        gnc::MotorState ms;
        ms.omega.setZero();

        double max_pos_err = 0.0;
        double max_vel = 0.0;

        const int N = static_cast<int>(Tsim / dt);
        for (int k = 0; k < N; ++k)
        {
            const double t = k * dt; // current simulation time

            // Reference
            gnc::Reference r = ref_fn(t);

            // Controller:state -> wrench
            gnc::CascadeOutput out = ctrl.update(s, r, dt);

            // Allocate -> omega_cmd
            Eigen::Vector4d u_cmd = out.wrench_cmd;
            Eigen::Vector4d u_ach_alloc;
            // Convert to the angular velocities of the four motor outputs
            Eigen::Vector4d omega_cmd = gnc::allocateOmegaCmd(cfg, mx, u_cmd, &u_ach_alloc);

            // Motor lag -> omega   , will change ms
            gnc::stepMotors(ms, mp, omega_cmd, dt);

            // Actual achieved wrench from actual omega (more realistic than using allocated)
            // Deduce the actual torque magnitude from the angular velocity
            Eigen::Vector4d u_ach = gnc::wrenchFromOmega(cfg, ms.omega);

            // Disturbance (test harness) : inject extra acceleration in NED
            //(industrial sim would add force term; for now we inject acceleration directly)
            s.v_n += disturbance_acc_n_fn(t) * dt;

            // Plant step
            gnc::ControlInput u;
            u.thrust = u_ach[0];
            u.tau_b = Eigen::Vector3d(u_ach[1], u_ach[2], u_ach[3]);
            gnc::stepKinematics(s, u, plant_params, dt);

            // Metrics
            const Eigen::Vector3d e_p = r.p_des - s.p_n;
            max_pos_err = std::max(max_pos_err, e_p.norm());
            max_vel = std::max(max_vel, s.v_n.norm());

            if (k % static_cast<int>(0.5 / dt) == 0)
            {
                std::cout << "t=" << t
                          << " |p|=" << s.p_n.norm()
                          << " |e_p|=" << e_p.norm()
                          << " |v|=" << s.v_n.norm()
                          << " T=" << u_ach[0]
                          << " tau=" << u.tau_b.transpose()
                          << "\n";
            }
        }
        std::cout << "Max position error norm: " << max_pos_err << " m\n";
        std::cout << "Max speed norm:          " << max_vel << " m/s\n";
        std::cout << "Final p: " << s.p_n.transpose() << "\n";
        std::cout << "Final v: " << s.v_n.transpose() << "\n";
    }

    // geometery attitude controller module
    AttMetrics runGeomAttitudeFlipTest(
        const char *csv_path,
        gnc::RigidBodyState s,
        gnc::KinematicsParams plant,
        const gnc::GeomAttController &gc,
        double dt, double Tsim,
        const Eigen::Matrix3d &Rd)
    {
        std::ofstream ofs(csv_path);
        ofs << "t,eR_norm,psi,omega_norm,ortho_err,det_err,tau_x,tau_y,tau_z\n";

        AttMetrics m;
        const double settle_th = 0.05;  // rad-ish:threshold on ||eR||
        const double settle_hold = 0.5; // seconds
        double settle_acc = 0.0;

        const double T_hover = plant.mass * plant.gravity;

        for (int k = 0; k < static_cast<int>(Tsim / dt); ++k)
        {
            const double t = k * dt;
            const Eigen::Matrix3d R = gnc::quatToRbn(s.q_bn);

            // Desired rates are zero for this experiment
            const Eigen::Vector3d omega_d = Eigen::Vector3d::Zero();
            const Eigen::Vector3d omega_d_dot = Eigen::Vector3d::Zero();

            auto out = gc.update(R, s.omega_b, Rd, omega_d, omega_d_dot);

            // Plant step
            gnc::ControlInput u;
            u.thrust = T_hover;
            u.tau_b = out.tau;
            gnc::stepKinematics(s, u, plant, dt);

            // Metrics
            const double eR_norm = out.eR.norm();
            const double psi = out.psi;
            const double omega_norm = s.omega_b.norm();

            const double ortho = gnc::orthogonalityError(R);
            const double dete = gnc::detError(R);

            m.max_eR = std::max(m.max_eR, eR_norm);
            m.max_psi = std::max(m.max_psi, psi);
            m.max_omega = std::max(m.max_omega, omega_norm);
            m.max_ortho = std::max(m.max_ortho, ortho);
            m.max_det = std::max(m.max_det, dete);

            // settling time: eR stays below threshold for settle_hold
            if (eR_norm < settle_th)
            {
                settle_acc += dt;
                if (m.settle_time < 0.0 && settle_acc >= settle_hold)
                {
                    m.settle_time = t - settle_hold;
                }
                else
                {
                    settle_acc = 0.0;
                }
                ofs << t << "," << eR_norm << "," << psi << "," << omega_norm << ","
                    << ortho << "," << dete << ","
                    << out.tau.x() << "," << out.tau.y() << "," << out.tau.z() << "\n";
            }
        }

        return m;
    }
}