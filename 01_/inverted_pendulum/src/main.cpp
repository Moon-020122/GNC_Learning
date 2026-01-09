#include <iostream>
#include <cmath>

#include "pendulum_dynamics.hpp"
#include "pendulum_params.hpp"
#include "integrators.hpp"
#include "controllers.hpp"
#include "logger.hpp"
#include "linear_discrete_model.hpp"
#include "plant_linear.hpp"
#include "saturations.hpp"
#include "pid.hpp"
#include "cascade_pid.hpp"
#include "observers.hpp"
#include "pendulum_ekf_model.hpp"
#include "ekf.hpp"
#include "random"

int main()
{

    try
    {
        const bool use_linear_plant = false;
        const bool use_observer = true;
        enum class ControlMode
        {
            Lqr,
            Pid,
            CascadePid
        };
        enum class ObserversMode
        {
            Luen,
            klam,
            Exklam
        };
        const ControlMode control_mode = ControlMode::Lqr;
        const ObserversMode observers_mode = ObserversMode::klam;
        LinearDiscreteModel lm;
        // simulation settings
        const double dt = 0.01;
        const double t_end = 20;
        const int steps = static_cast<int>(t_end / dt);

        // initial condition
        State x = State::Zero();
        x(0) = 0.0; // x
        x(1) = 0.0;
        x(2) = deg2rad(5.0); // theta
        x(3) = 0.0;

        PendulumParams p; // using default setting
        p.b_x = 0.2;
        p.c_th = 0.5;

        lm.Ad << 1.0000, 0.0100, -0.0002, -0.0000,
            0, 1.0000, -0.0392, -0.0002,
            0, 0, 1.0014, 0.0100,
            0, 0, 0.2745, 1.0014;
        lm.Bd << 0.0001,
            0.0200,
            -0.0002,
            -0.0400;
        lm.C << 1, 0, 0, 0,
            0, 0, 1, 0;

        LinearDiscretePlant linplant(lm);

        ObserverModel om;
        om.Ad = lm.Ad;
        om.Bd = lm.Bd;
        om.C = lm.C;

        LuenbergerObserver obs;
        obs.m = om;
        obs.L << 0.3668, 0.0093,
            3.3563, 0.1335,
            0.0092, 0.3759,
            0.1723, 3.7978;

        SteadyKalmanFilter kf;
        kf.Kk << 0.9903, -0.0000,
            0.9854, -0.0115,
            -0.0000, 0.9903,
            -0.0110, 1.1325;

        PendulumEkfModel ekf_model(p, dt);
        EKF ekf(ekf_model);
        Mat4 P0 = Mat4::Zero();
        P0.diagonal() << 1e-4, 1e-1, 1e-4, 1e-1;
        Mat4 Q = Mat4::Zero();
        Q.diagonal() << 1e-8, 1e-6, 1e-8, 1e-6;
        ekf.set_Q(Q);
        Mat2 R = Mat2::Zero();
        R.diagonal() << 1e-4, 1e-4;
        ekf.set_R(R);

        State xhat0 = State::Zero();
        xhat0(0) = x(0);
        xhat0(2) = x(2);
        obs.reset(xhat0);
        kf.reset(xhat0);
        ekf.reset(xhat0, P0);

        // measurement noise generator
        std::mt19937 rng(0);
        std::normal_distribution<double> nx(0.0, std::sqrt(R(0, 0)));  // position
        std::normal_distribution<double> nth(0.0, std::sqrt(R(1, 1))); // angle

        linplant.set_state(x);
        // Controller

        LQRController ctrl;
        ctrl.u_min = p.u_min;
        ctrl.u_max = p.u_max;
        ctrl.K << -3.1623, -4.9645, -45.4799, -9.6556;
        ctrl.x_ref = State::Zero();

        PidGains g;
        g.Kp = 10.0;
        g.Ki = 0.0;
        g.Kd = 2.0;
        AnglePidController pid_ctrl{PID(g, dt, p.u_min, p.u_max), 0.0};
        pid_ctrl.pid.set_derivative_filter_tau(0.02);

        PidGains pos_g;
        pos_g.Kp = 0.1;
        pos_g.Ki = 0.0;
        pos_g.Kd = 0.01;
        PID pos_pid(pos_g, dt, -deg2rad(10.0), +deg2rad(10.0));
        pos_pid.set_derivative_filter_tau(0.05);
        PidGains ang_g;
        ang_g.Kp = 50.0;
        ang_g.Ki = 0.0;
        ang_g.Kd = 3.0;
        PID ang_pid(ang_g, dt, p.u_min, p.u_max);
        ang_pid.set_derivative_filter_tau(0.02);
        CascadePidConfig cascade_cfg; // use default settings
        CascadePidController cascade_ctrl{
            pos_pid,
            ang_pid,
            cascade_cfg,
            dt};
        cascade_ctrl.set_x_ref(0.0);
        cascade_ctrl.reset();

        // logger
        CsvLogger logger("data/run.csv");
        logger.write_header();

        // main loop: discrete-time control, continuous-time plant integration
        double t = 0.0;
        double u_prev = 0.0;
        for (int k = 0; k <= steps; ++k)
        {
            Meas y;
            y << x(0), x(2);
            //  accompanied by noise
            Meas z;
            z(0) = x(0) + nx(rng);
            z(1) = x(2) + nth(rng);
            switch (observers_mode)
            {
            case ObserversMode::Luen:
                obs.update(u_prev, z);
                break;
            case ObserversMode::klam:
                kf.update(u_prev, z);
                break;
            case ObserversMode::Exklam:
                // EKF
                ekf.predict(u_prev);
                ekf.update(z);
                break;
            }

            State x_ctrl = x;
            switch (observers_mode)
            {
            case ObserversMode::Luen:
                x_ctrl = obs.xhat;
                break;
            case ObserversMode::klam:
                x_ctrl = kf.xhat;
                break;
            case ObserversMode::Exklam:
                x_ctrl = ekf.xhat(); // The function returns private member variables.
                // EKF
                break;
            }

            double u = 0.0;
            double theta_ref_dbg = 0.0;
            switch (control_mode)
            {
            case ControlMode::Lqr:
                u = ctrl.compute(x_ctrl);
                break;
            case ControlMode::Pid:
                u = pid_ctrl.compute(x_ctrl);
                break;
            case ControlMode::CascadePid:
                u = cascade_ctrl.compute(x_ctrl);
                theta_ref_dbg = cascade_ctrl.theta_ref(); // log theta_ref
                break;
            }
            logger.log(t, x, u, theta_ref_dbg, x_ctrl(0), x_ctrl(2));
            if (use_linear_plant)
            {
                linplant.step(u);
                x = linplant.state();
            }
            else
            {
                x = rk4_step(x, u, dt, p);
            }
            u_prev = u;
            t += dt;
        }
        std::cout << "Done\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << "ERROR:" << e.what() << "\n";
        return 1;
    }
}
