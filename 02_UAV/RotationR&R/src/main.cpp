#include <iostream>
#include <random>
#include <cmath>

#include "rotation.hpp"

static double deg2rad(double deg) { return deg * M_PI / 180.0; }

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
        //If a disturbance is added, it will fail the check.
        // Eigen::Quaterniond q_norm = gnc::normalizeQuat(q_drift);
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

        return pass
        ? 0
        : 1;
}