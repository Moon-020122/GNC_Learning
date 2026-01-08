#pragma once

struct PendulumParams{
    double M{0.5};
    double m{0.2};
    double l{0.5};
    double g{9.80};

    double u_min{-10.0};
    double u_max{+10.0};

    double b_x{0.0}; //cart viscous friction (N per (m/s))
    double c_th{0.0}; //simple angular domping (1/s),used as thdd - = c_th * thdot
};