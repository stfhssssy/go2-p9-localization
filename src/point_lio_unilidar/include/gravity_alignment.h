#pragma once

#include <cmath>
#include <Eigen/Geometry>

namespace point_lio {

// R_WI maps a gravity direction in IMU axes into the chosen world axes.
// Its yaw is not determined by gravity. Leave the output unchanged on failure.
inline bool gravity_alignment_rotation(const Eigen::Vector3d& gravity_in_imu,
    const Eigen::Vector3d& gravity_in_world, Eigen::Matrix3d& world_from_imu)
{
    if (!gravity_in_imu.allFinite() || !gravity_in_world.allFinite()) {
        return false;
    }
    const double imu_norm = gravity_in_imu.stableNorm();
    const double world_norm = gravity_in_world.stableNorm();
    if (!std::isfinite(imu_norm) || !std::isfinite(world_norm)
        || imu_norm < 1e-6 || world_norm < 1e-6) {
        return false;
    }
    // FromTwoVectors handles antiparallel vectors with a proper 180-degree
    // rotation. -Identity is a reflection (det=-1), not a valid SO(3) state.
    world_from_imu = Eigen::Quaterniond::FromTwoVectors(
        gravity_in_imu / imu_norm, gravity_in_world / world_norm)
        .normalized().toRotationMatrix();
    return true;
}

}  // namespace point_lio
