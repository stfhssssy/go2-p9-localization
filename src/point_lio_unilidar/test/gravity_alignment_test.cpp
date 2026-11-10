#include "gravity_alignment.h"

#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>

namespace {
int checked = 0;
double maximum_alignment_error = 0.;

void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

void check(const Eigen::Vector3d& source, const Eigen::Vector3d& target)
{
    Eigen::Matrix3d r;
    require(point_lio::gravity_alignment_rotation(source, target, r), "valid input rejected");
    require(r.allFinite(), "rotation is not finite");
    require(std::abs(r.determinant()-1.) < 1e-10, "rotation is not proper SO(3)");
    require((r.transpose()*r-Eigen::Matrix3d::Identity()).norm() < 1e-10,
            "rotation is not orthogonal");
    const double error = (r*source.normalized()-target.normalized()).norm();
    maximum_alignment_error = std::max(maximum_alignment_error, error);
    require(error < 1e-7, "rotation maps gravity in the wrong direction");
    // The estimator initializes body specific force as -R^T*g_world.
    // Its stationary world acceleration must cancel gravity exactly.
    const Eigen::Vector3d specific_force = -r.transpose()*target;
    require((r*specific_force+target).norm() < 1e-10*target.norm(),
            "stationary propagation does not cancel gravity");
    ++checked;
}

void reject(const Eigen::Vector3d& source, const Eigen::Vector3d& target)
{
    Eigen::Matrix3d r = 2.*Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d before = r;
    require(!point_lio::gravity_alignment_rotation(source, target, r), "invalid input accepted");
    require(r == before, "invalid input modified the caller's rotation");
    ++checked;
}
}  // namespace

int main()
{
    try {
        const Eigen::Vector3d down(0., 0., -9.81);
        Eigen::Matrix3d identity;
        require(point_lio::gravity_alignment_rotation(down, down, identity), "same direction failed");
        require(identity.isApprox(Eigen::Matrix3d::Identity(), 1e-12), "same direction is not identity");
        check(down, down);
        check(-down, down);
        check(Eigen::Vector3d(1e-10, 0., 9.81), down);
        check(Eigen::Vector3d(0., -9.81, 0.), down);
        check(Eigen::Vector3d(3., 4., -8.), down);
        check(2.*down, down);  // alignment changes direction, not magnitude

        std::mt19937 generator(20261109);
        std::normal_distribution<double> normal(0., 1.);
        for (int i = 0; i < 2000; ++i) {
            Eigen::Vector3d a(normal(generator), normal(generator), normal(generator));
            Eigen::Vector3d b(normal(generator), normal(generator), normal(generator));
            a = a.normalized()*9.81;
            b = b.normalized()*9.81;
            check(a, b);
            check(a, -a);  // antiparallel about arbitrary axes, not only Z
        }
        reject(Eigen::Vector3d::Zero(), down);
        reject(down, Eigen::Vector3d::Zero());
        reject(Eigen::Vector3d(1e-8, 0., 0.), down);
        reject(down, Eigen::Vector3d(0., 1e-8, 0.));
        Eigen::Vector3d invalid = down;
        invalid.x() = std::numeric_limits<double>::quiet_NaN();
        reject(invalid, down);
        reject(down, invalid);
        invalid.x() = std::numeric_limits<double>::infinity();
        reject(invalid, down);
        reject(down, invalid);
        std::cout << "PASS: " << checked << " cases; maximum normalized alignment error="
                  << maximum_alignment_error << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
