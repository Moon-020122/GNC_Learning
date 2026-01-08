#pragma once
#include <fstream>
#include <string>
#include "pendulum_dynamics.hpp"

class CsvLogger
{
public:
    explicit CsvLogger(const std::string &path);
    void write_header();
    void log(double t, const State &x, double u, double theta_ref, double xhat, double thetahat);

private:
    std::ofstream ofs_;
};