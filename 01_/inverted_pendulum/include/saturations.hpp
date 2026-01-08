#pragma once
#include <algorithm>

inline double clamp(double v, double lo, double hi) {
    return std::max(lo,std::min(v,hi));
}


constexpr double deg2rad(double deg)
{
    return deg * 3.14159265358979323846 / 180.0;
}