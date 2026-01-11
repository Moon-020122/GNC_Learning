#pragma once
#include "linear_discrete_model.hpp"

class LinearDiscretePlant
{
public:
    explicit LinearDiscretePlant(const LinearDiscreteModel &m) : m_(m) {};

    void set_state(const State &x0) { x_ = x0; }
    const State &state() const { return x_; }

    void step(double u)
    {
        x_ = m_.Ad * x_ + m_.Bd * u;
    }

private:
    LinearDiscreteModel m_;
    State x_{State::Zero()};
};