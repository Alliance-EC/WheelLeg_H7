#pragma once

#include "app/system_parameters.hpp"
#include <cmath>

namespace app::observer {
class velocity_kalman {
public:
    velocity_kalman() { speed_kalmanfilter_init(); }
    ~velocity_kalman() = default;
    // z[2]=[v,a]
    double update(const double z[2]) {
        if (std::abs(z[0]) > 1e2 || std::abs(z[1]) > 1e2 || std::isnan(z[0]) || std::isnan(z[1]))
            return 0;

        if (Is_parking) {
            sigma_v_ = sigma_v_parking;
        } else {
            sigma_v_ = sigma_v_normal;
        }
        velocity_est = update_KF(z);
        return velocity_est;
    }
    void set_parking() { Is_parking = true; }
    void set_normal() {
        Is_slip_   = false;
        Is_parking = false;
    }
    [[nodiscard]] bool get_slip() const { return Is_slip_; };
    double velocity_est = 0;

private:
    double x_est_[2];
    double p_est_[4];

    static constexpr double dt              = app::dt;
    double sigma_q_                         = 1;
    double sigma_v_                         = 0.75;
    static constexpr double sigma_v_normal  = 0.75;
    static constexpr double sigma_v_parking = 0.1;
    static constexpr double sigma_a_        = 1;

    bool Is_slip_   = false; // not using yet
    bool Is_parking = false;

    double update_KF(const double z[2]) {
        static const int a[4]{1, 0, 0, 1};
        double A[4];
        double W[4];
        double b_A[4];
        double p_prd[4];
        double x_prd[2];
        double a21;
        double a22;
        double a22_tmp;
        double d;
        double d1;
        double d2;
        double d3;
        int r1;
        int r2;
        A[0] = 1.0;
        A[2] = dt;
        A[1] = 0.0;
        A[3] = 1.0;
        //      % [v]
        //  [a]
        // 测量矩阵
        // 过程噪声系数
        // 过程噪声协方差
        // 测量噪声协方差
        //  Initial state conditions
        //  Predicted state and covariance
        a21     = sigma_q_ * sigma_q_;
        d       = x_est_[0];
        d1      = x_est_[1];
        d2      = p_est_[0];
        d3      = p_est_[1];
        a22_tmp = p_est_[2];
        a22     = p_est_[3];
        for (r1 = 0; r1 < 2; r1++) {
            double d4;
            double d5;
            r2            = static_cast<int>(A[r1]);
            d4            = A[r1 + 2];
            d5            = static_cast<double>(r2) * d2 + d4 * d3;
            x_prd[r1]     = static_cast<double>(r2) * d + d4 * d1;
            d4            = static_cast<double>(r2) * a22_tmp + d4 * a22;
            p_prd[r1]     = d5 + d4 * dt;
            p_prd[r1 + 2] = d5 * 0.0 + d4;
        }
        p_prd[0] += 0.5 * (dt * dt) * a21;
        p_prd[3] += a21 * dt;
        //  Estimation
        d  = p_prd[0];
        d1 = p_prd[2];
        d2 = p_prd[1];
        d3 = p_prd[3];
        for (r1 = 0; r1 < 2; r1++) {
            int i;
            r2        = a[r1];
            i         = a[r1 + 2];
            a22_tmp   = static_cast<double>(r2) * d + static_cast<double>(i) * d1;
            A[r1]     = a22_tmp;
            a22       = static_cast<double>(r2) * d2 + static_cast<double>(i) * d3;
            A[r1 + 2] = a22;
            W[r1]     = a22_tmp + a22 * 0.0;
            W[r1 + 2] = a22_tmp * 0.0 + a22;
        }
        W[0] += sigma_v_ * sigma_v_;
        W[3] += sigma_a_ * sigma_a_;
        if (std::abs(W[1]) > std::abs(W[0])) {
            r1 = 1;
            r2 = 0;
        } else {
            r1 = 0;
            r2 = 1;
        }
        a21     = W[r2] / W[r1];
        a22_tmp = W[r1 + 2];
        a22     = W[r2 + 2] - a21 * a22_tmp;
        d       = (A[r2] - A[r1] * a21) / a22;
        b_A[1]  = d;
        b_A[0]  = (A[r1] - d * a22_tmp) / W[r1];
        d1      = A[r1 + 2];
        d       = (A[r2 + 2] - d1 * a21) / a22;
        //  Estimated state and covariance
        A[0]    = b_A[0];
        A[1]    = (d1 - d * a22_tmp) / W[r1];
        a21     = z[0] - (x_prd[0] + 0.0 * x_prd[1]);
        A[2]    = b_A[1];
        A[3]    = d;
        a22_tmp = z[1] - (0.0 * x_prd[0] + x_prd[1]);
        for (r1 = 0; r1 < 2; r1++) {
            d              = A[r1 + 2];
            d1             = A[r1];
            d2             = d1 + d * 0.0;
            d3             = d1 * 0.0 + d;
            x_est_[r1]     = x_prd[r1] + (d1 * a21 + d * a22_tmp);
            p_est_[r1]     = p_prd[r1] - (d2 * p_prd[0] + d3 * p_prd[1]);
            p_est_[r1 + 2] = p_prd[r1 + 2] - (d2 * p_prd[2] + d3 * p_prd[3]);
        }
        //  Compute the estimated measurements
        return x_est_[0] + 0.0 * x_est_[1];
    }

    //
    // Initialize state transition matrix
    //
    // Arguments    : void
    // Return Type  : void
    //
    void speed_kalmanfilter_init() {
        p_est_[0] = 1000.0;
        p_est_[1] = 0.0;
        p_est_[2] = 0.0;
        p_est_[3] = 1000.0;
        x_est_[0] = 0.0;
        x_est_[1] = 0.0;
        //  x_est=[v,a]'
    }
};
} // namespace app::observer