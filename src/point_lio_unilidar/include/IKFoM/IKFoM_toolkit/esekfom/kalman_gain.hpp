#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>

namespace esekfom {
// The lidar Jacobian observes only the first twelve state coordinates.
// Gazebo snapshots can contain thousands of measurements at the same instant.
// Use the equivalent information form for such batches instead of inverting
// a measurements-by-measurements matrix (cubic in the scan size).
template<typename Scalar, int N>
Eigen::Matrix<Scalar, N, Eigen::Dynamic> lidar_kalman_gain(
    const Eigen::Matrix<Scalar, N, N>& prior,
    const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& h,
    Scalar noise)
{
    using StateMatrix = Eigen::Matrix<Scalar, N, N>;
    if (h.rows() <= N) {
        Eigen::Matrix<Scalar, N, Eigen::Dynamic> pht = prior.template leftCols<12>() * h.transpose();
        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> innovation = h * pht.template topRows<12>();
        innovation.diagonal().array() += noise;
        return innovation.ldlt().solve(pht.transpose()).transpose();
    }
    // Whiten by a square root of P instead of explicitly forming P^-1.
    // This retains weakly observed directions without inverting tiny variances.
    Eigen::LLT<StateMatrix> prior_root(prior);
    if (prior_root.info() != Eigen::Success)
        throw std::runtime_error("LiDAR prior covariance is not positive definite");
    const StateMatrix lower = prior_root.matrixL();
    const Eigen::Matrix<Scalar, Eigen::Dynamic, N> whitened = h * lower.template topRows<12>() / std::sqrt(noise);
    const StateMatrix system = StateMatrix::Identity() + whitened.transpose()*whitened;
    return lower * system.llt().solve(whitened.transpose()) / std::sqrt(noise);
}

template<typename Scalar, int N>
Eigen::Matrix<Scalar, N, N> lidar_posterior_covariance(
    const Eigen::Matrix<Scalar, N, N>& prior,
    const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& h,
    const Eigen::Matrix<Scalar, N, Eigen::Dynamic>& gain, Scalar noise)
{
    using StateMatrix = Eigen::Matrix<Scalar, N, N>;
    StateMatrix residual = StateMatrix::Identity();
    residual.template leftCols<12>().noalias() -= gain * h;
    // Joseph form avoids cancellation in P - K H P and preserves PSD.
    StateMatrix result = residual*prior*residual.transpose() + noise*gain*gain.transpose();
    return (result + result.transpose()).eval() * Scalar(0.5);
}
}
