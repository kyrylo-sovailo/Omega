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
        const Eigen::Matrix<T, X, Y> K = (C * P * C.transport() + R).colPivHouseholderQr().solve(P * C.transpose());
        x = x + K * (y - C * x);
        P = (Eigen::Matrix<T, X, X>::Identity() - K * C) * P * (Eigen::Matrix<T, X, X>::Identity() - K * C).transpose() + K * R * K.transpose(); 
    }
}