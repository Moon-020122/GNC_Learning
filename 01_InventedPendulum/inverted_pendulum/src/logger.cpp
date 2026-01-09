#include "logger.hpp"
#include <stdexcept>

CsvLogger::CsvLogger(const std::string &path) : ofs_(path)
{
    if (!ofs_)
    {
        throw std::runtime_error("Failed to open log file:" + path);
    }
}

void CsvLogger::write_header()
{
    ofs_ << "t,x,xdot,theta,thetadot,u,theta_ref,x_hat,theta_hat\n";
}

void CsvLogger::log(double t, const State &x, double u, double theta_ref, double xhat, double thetahat)
{
    ofs_
        << t << ","
        << x(0) << "," << x(1) << ","
        << x(2) << "," << x(3) << ","
        << u << "," << theta_ref << ","
        << xhat << "," << thetahat << "\n";
}