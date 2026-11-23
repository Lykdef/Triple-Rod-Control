#include "triple/automatic_controller.hpp"

#include "triple/swing_up_data.hpp"

#include <algorithm>
#include <cmath>

namespace {

constexpr double pi = 3.14159265358979323846;

double wrapAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

// Mechanical energy of the chain relative to the moving cart. Excluding cart
// translation prevents the pump from storing the target energy as cart speed.
double chainEnergy(const State& state, const Parameters& parameters) {
    const std::size_t n = parameters.linkCount();
    const Eigen::VectorXd omega = state.tail(n);
    const Eigen::MatrixXd jointMass = massMatrix(state, parameters).block(1, 1, n, n);
    double energy = 0.5 * omega.dot(jointMass * omega);

    double baseHeight = 0.0;
    for (std::size_t rod = 0; rod < n; ++rod) {
        const double length = parameters.rodLengths[rod];
        const double centreHeight = baseHeight
            + 0.5 * length * std::cos(state(1 + rod));
        energy += parameters.rodMasses[rod] * parameters.gravity * centreHeight;
        baseHeight += length * std::cos(state(1 + rod));
    }
    return energy;
}

bool isMotionlessHanging(const State& state, std::size_t n) {
    if (std::abs(state(0)) > 1.0e-9
        || state.tail(n + 1).cwiseAbs().maxCoeff() > 1.0e-9) {
        return false;
    }
    for (std::size_t rod = 0; rod < n; ++rod) {
        if (std::abs(std::abs(wrapAngle(state(1 + rod))) - pi) > 1.0e-9) {
            return false;
        }
    }
    return true;
}

} // namespace

AutomaticController::AutomaticController(
    const Parameters& parameters,
    double sampleTime)
    : parameters_(parameters),
      lqr_(parameters, sampleTime),
      sampleTime_(sampleTime) {
}

void AutomaticController::reset(const State& initialState) {
    const std::size_t n = parameters_.linkCount();
    double maxAngle = 0.0;
    for (std::size_t rod = 0; rod < n; ++rod) {
        maxAngle = std::max(
            maxAngle, std::abs(wrapAngle(initialState(1 + rod))));
    }

    const bool hanging = isMotionlessHanging(initialState, n);
    swingingUp_ = hanging || maxAngle > 1.0;
    trajectoryTime_ = 0.0;
    useTrajectory_ = hanging && swingUpTrajectory(n) != nullptr;
}

double AutomaticController::uprightEnergy() const {
    double energy = 0.0;
    double baseHeight = 0.0;
    for (std::size_t rod = 0; rod < parameters_.linkCount(); ++rod) {
        const double length = parameters_.rodLengths[rod];
        energy += parameters_.rodMasses[rod] * parameters_.gravity
            * (baseHeight + 0.5 * length);
        baseHeight += length;
    }
    return energy;
}

double AutomaticController::trajectoryForce(const State& measurement) const {
    const SwingUpTrajectory& trajectory =
        *swingUpTrajectory(parameters_.linkCount());
    const std::size_t steps = trajectory.forces.size();
    const std::size_t stateSize = trajectory.stateSize;
    const double position = std::min(
        trajectoryTime_ / swingUpTimeStep, static_cast<double>(steps - 1));
    const std::size_t index = static_cast<std::size_t>(position);
    const double fraction = position - static_cast<double>(index);
    const std::size_t nextForce = std::min(index + 1, steps - 1);

    using ConstMap = Eigen::Map<const Eigen::VectorXd>;
    const ConstMap firstState(
        trajectory.states.data() + index * stateSize, stateSize);
    const ConstMap nextState(
        trajectory.states.data() + (index + 1) * stateSize, stateSize);
    const State reference = firstState * (1.0 - fraction) + nextState * fraction;

    const std::size_t nextGain = std::min(index + 1, steps - 1);
    const ConstMap firstGain(
        trajectory.feedbackGains.data() + index * stateSize, stateSize);
    const ConstMap nextGainRow(
        trajectory.feedbackGains.data() + nextGain * stateSize, stateSize);
    const State gain = firstGain * (1.0 - fraction) + nextGainRow * fraction;

    State error = measurement - reference;
    for (std::size_t rod = 0; rod < parameters_.linkCount(); ++rod) {
        error(1 + rod) = wrapAngle(error(1 + rod));
    }

    const double feedforward = trajectory.forces[index] * (1.0 - fraction)
        + trajectory.forces[nextForce] * fraction;
    return feedforward + gain.dot(error);
}

double AutomaticController::swingUpForce(const State& measurement) const {
    const std::size_t n = parameters_.linkCount();
    const double swingVelocity = measurement(n + 2)
        * std::cos(measurement(1));
    const double energyError = chainEnergy(measurement, parameters_)
        - uprightEnergy();

    // At the motionless downward equilibrium, a small kick starts pumping.
    double drive = 0.0;
    if (std::abs(swingVelocity) < 0.03
        && std::abs(measurement(n + 1)) < 0.05) {
        drive = measurement(0) > 0.0 ? -1.0 : 1.0;
    } else {
        drive = std::tanh(1.5 * swingVelocity);
    }

    const double energyGain = 60.0 / static_cast<double>(n * n);
    const double energyForce = energyGain * energyError * drive;
    const double centeringForce = -0.8 * measurement(0)
        - 1.2 * measurement(n + 1);
    return energyForce + centeringForce;
}

double AutomaticController::computeForce(const State& measurement) {
    const std::size_t n = parameters_.linkCount();
    double maxAngle = 0.0;
    double maxAngularVelocity = 0.0;
    for (std::size_t rod = 0; rod < n; ++rod) {
        maxAngle = std::max(maxAngle, std::abs(wrapAngle(measurement(1 + rod))));
        maxAngularVelocity = std::max(
            maxAngularVelocity, std::abs(measurement(n + 2 + rod)));
    }

    if (swingingUp_) {
        const double catchAngle = n == 3 ? 0.65 : 0.4;
        if (maxAngle < catchAngle && maxAngularVelocity < 3.0) {
            swingingUp_ = false;
            useTrajectory_ = false;
        }
    } else if (maxAngle > 1.0 || maxAngularVelocity > 8.0) {
        swingingUp_ = true;
        useTrajectory_ = false;
    }

    if (!swingingUp_) {
        return lqr_.computeForce(measurement);
    }

    if (useTrajectory_) {
        const double force = trajectoryForce(measurement);
        trajectoryTime_ += sampleTime_;
        return force;
    }
    return swingUpForce(measurement);
}
