#pragma once
#include <algorithm>

class RateLimiter
{
public:
    RateLimiter(double max_rate, double init = 0.0)
        : max_rate_(max_rate), y_(init) {}
void reset(double y0 = 0.0) {y_ = y0;}

double update (double target, double dt){
    const double max_step = max_rate_ * dt;
    const double lo = y_ - max_step;
    const double hi = y_ + max_step;
    y_ = std::max(lo,std::min(target, hi));
    return y_;
}


private:
    double max_rate_;
    double y_;
};