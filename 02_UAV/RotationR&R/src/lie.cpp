#include "lie.hpp"
#include <cmath>

namespace gnc
{

    Eigen::Matrix3d hat(const Eigen::Vector3d &w)
    {
        Eigen::Matrix3d W;
        W << 0.0, -w.z(), w.y(),
            w.z(), 0.0, -w.x(),
            -w.y(), w.x(), 0.0;
        return W;
    }

    Eigen::Vector3d vee(const Eigen::Matrix3d& W)
    {
        return Eigen::Vector3d(W(2, 1), W(0, 2), W(1, 0));
    }

    Eigen::Vector3d so3Error(const Eigen::Matrix3d &R,
                             const Eigen::Matrix3d &Rd)
    {
        const Eigen::Matrix3d E = Rd.transpose() * R - R.transpose() * Rd;
        return 0.5 * vee(E);
    }

    double so3Psi(const Eigen::Matrix3d &R,
                  const Eigen::Matrix3d &Rd)
    {
        const double tr = (Rd.transpose() * R).trace();
        return 0.5 * (3.0 - tr);
    }
}