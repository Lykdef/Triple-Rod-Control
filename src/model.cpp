#include "triple/model.hpp"

#include <cmath>
#include <numeric>
#include <stdexcept>

namespace {

// Length of rod j's contribution to rod i's centre of mass.
// It is zero when rod j is above rod i in the serial chain.
double effectiveLength(
    const Parameters& p,
    std::size_t rod,
    std::size_t joint) {
    if (joint > rod) {
        return 0.0;
    }
    if (joint == rod) {
        return 0.5 * p.rodLengths[rod];
    }
    return p.rodLengths[joint];
}

double pivotCoupling(const Parameters& p, std::size_t joint) {
    double result = 0.0;
    for (std::size_t rod = 0; rod < p.linkCount(); ++rod) {
        result += p.rodMasses[rod] * effectiveLength(p, rod, joint);
    }
    return result;
}

double jointCoupling(
    const Parameters& p,
    std::size_t first,
    std::size_t second) {
    double result = 0.0;
    for (std::size_t rod = 0; rod < p.linkCount(); ++rod) {
        result += p.rodMasses[rod]
            * effectiveLength(p, rod, first)
            * effectiveLength(p, rod, second);
    }
    return result;
}

} // namespace

void Parameters::setLinkCount(std::size_t count) {
    if (count == 0) {
        throw std::invalid_argument("The chain must contain at least one rod.");
    }

    const double mass = rodMasses.empty() ? 0.2 : rodMasses.front();
    const double length = rodLengths.empty() ? 0.6 : rodLengths.front();
    rodMasses.assign(count, mass);
    rodLengths.assign(count, length);
}

Eigen::MatrixXd massMatrix(const State& y, const Parameters& p) {
    const std::size_t n = p.linkCount();
    const std::size_t size = n + 1;
    Eigen::MatrixXd mass = Eigen::MatrixXd::Zero(size, size);

    mass(0, 0) = p.cartMass + std::accumulate(
        p.rodMasses.begin(), p.rodMasses.end(), 0.0);

    for (std::size_t joint = 0; joint < n; ++joint) {
        const double coupling = pivotCoupling(p, joint);
        const double theta = y(1 + joint);
        mass(0, 1 + joint) = coupling * std::cos(theta);
        mass(1 + joint, 0) = mass(0, 1 + joint);
    }

    for (std::size_t first = 0; first < n; ++first) {
        for (std::size_t second = 0; second < n; ++second) {
            const double angleDifference =
                y(1 + first) - y(1 + second);
            double value = jointCoupling(p, first, second)
                * std::cos(angleDifference);

            if (first == second) {
                const double length = p.rodLengths[first];
                value += p.rodMasses[first] * length * length / 12.0;
            }
            mass(1 + first, 1 + second) = value;
        }
    }

    return mass;
}

State derivative(const State& y, double force, const Parameters& p) {
    const std::size_t n = p.linkCount();
    const double cartVelocity = y(n + 1);
    const Eigen::VectorXd angularVelocities = y.tail(n);

    Eigen::VectorXd rhs = Eigen::VectorXd::Zero(n + 1);
    rhs(0) = force - p.cartFriction * cartVelocity;

    // Generalized forces contain centrifugal, Coriolis, gravity and damping.
    for (std::size_t rod = 0; rod < n; ++rod) {
        for (std::size_t joint = 0; joint < n; ++joint) {
            const double length = effectiveLength(p, rod, joint);
            const double theta = y(1 + joint);
            const double omega = angularVelocities(joint);
            rhs(0) += p.rodMasses[rod] * length
                * std::sin(theta) * omega * omega;
        }
    }

    for (std::size_t first = 0; first < n; ++first) {
        const double theta = y(1 + first);
        const double omega = angularVelocities(first);
        const double coupling = pivotCoupling(p, first);

        rhs(1 + first) = p.gravity * coupling * std::sin(theta)
            - p.jointDamping * omega;

        for (std::size_t second = 0; second < n; ++second) {
            const double angleDifference = theta - y(1 + second);
            const double otherOmega = angularVelocities(second);
            rhs(1 + first) -= jointCoupling(p, first, second)
                * std::sin(angleDifference) * otherOmega * otherOmega;
        }
    }

    const Eigen::VectorXd accelerations = massMatrix(y, p).ldlt().solve(rhs);
    State result(n + 1 + n + 1);
    result << cartVelocity, angularVelocities, accelerations;
    return result;
}

double totalEnergy(const State& y, const Parameters& p) {
    const std::size_t n = p.linkCount();
    const double cartVelocity = y(n + 1);

    double energy = 0.5 * p.cartMass * cartVelocity * cartVelocity;
    double baseVelocityX = cartVelocity;
    double baseVelocityY = 0.0;
    double baseHeight = 0.0;

    for (std::size_t rod = 0; rod < n; ++rod) {
        const double mass = p.rodMasses[rod];
        const double length = p.rodLengths[rod];
        const double theta = y(1 + rod);
        const double omega = y(n + 2 + rod);
        const double halfLength = 0.5 * length;
        const double cosine = std::cos(theta);
        const double sine = std::sin(theta);

        const double velocityX = baseVelocityX + halfLength * omega * cosine;
        const double velocityY = baseVelocityY - halfLength * omega * sine;
        const double centreHeight = baseHeight + halfLength * cosine;
        const double inertia = mass * length * length / 12.0;

        energy += 0.5 * mass * (velocityX * velocityX + velocityY * velocityY)
            + 0.5 * inertia * omega * omega
            + mass * p.gravity * centreHeight;

        baseVelocityX += length * omega * cosine;
        baseVelocityY -= length * omega * sine;
        baseHeight += length * cosine;
    }

    return energy;
}
