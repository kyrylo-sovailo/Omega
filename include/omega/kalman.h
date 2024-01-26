#pragma once
#include <Eigen/Dense>

namespace omega
{
    template <typename T, int X, int Y> void kalman_update(
        Eigen::Matrix<T, X, 1> &x,
        Eigen::Matrix<T, X, X> &P,
        const Eigen::Matrix<T, X, X> &A,
        const Eigen::Matrix<T, X, 1> &Bu,
        const Eigen::Matrix<T, X, X> &Q)
    {
        x = A * x + Bu;
        P = A * P * A.transpose() + Q;
    }

    template <typename T, int X, int Y> void kalman_correct(
        Eigen::Matrix<T, X, 1> &x,
        Eigen::Matrix<T, X, X> &P,
        const Eigen::Matrix<T, Y, 1> &y,
        const Eigen::Matrix<T, Y, X> &C,
        const Eigen::Matrix<T, Y, Y> &R)
    {
        const Eigen::Matrix<T, Y, Y> K_A = C * P * C.transpose() + R;
        const Eigen::Matrix<T, X, Y> K_b = P * C.transpose();
        const Eigen::Matrix<T, X, Y> K = K_A.transpose().colPivHouseholderQr().solve(K_b.transpose()).transpose(); //K * K_A = K_b <=> K_A^T * K^T = K_b^T
        x = x + K * (y - C * x);
        P = (Eigen::Matrix<T, X, X>::Identity() - K * C) * P * (Eigen::Matrix<T, X, X>::Identity() - K * C).transpose() + K * R * K.transpose(); 
    }
}