#include "triple/integrator.hpp"
#include "triple/model.hpp"
#include "triple/simulation.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void testSingleRodDynamicsRegression() {
    Parameters p;
    State y(4);
    y << 0.3, 0.2, -0.4, 0.7;
    const double force = 1.2;

    const double theta = y(1);
    const double omega = y(3);
    const double a = 0.5 * p.rodLengths[0];
    const double inertia = p.rodMasses[0] * p.rodLengths[0]
        * p.rodLengths[0] / 12.0 + p.rodMasses[0] * a * a;
    const double cartTerm = p.cartMass + p.rodMasses[0];
    const double coupling = p.rodMasses[0] * a * std::cos(theta);
    const double cartRhs = force + p.rodMasses[0] * a * omega * omega
        * std::sin(theta) - p.cartFriction * y(2);
    const double jointRhs = p.rodMasses[0] * p.gravity * a * std::sin(theta)
        - p.jointDamping * omega;
    const double determinant = cartTerm * inertia - coupling * coupling;

    State expected(4);
    expected << y(2), y(3),
        (inertia * cartRhs - coupling * jointRhs) / determinant,
        (-coupling * cartRhs + cartTerm * jointRhs) / determinant;
    const State actual = derivative(y, force, p);
    require(
        (actual - expected).cwiseAbs().maxCoeff() < 1.0e-10,
        "Generic dynamics no longer matches the original single-rod model.");
}

void testEnergyConservation(std::size_t linkCount) {
    Parameters parameters;
    parameters.setLinkCount(linkCount);
    parameters.cartFriction = 0.0;
    parameters.jointDamping = 0.0;
    parameters.trackMin = -100.0;
    parameters.trackMax = 100.0;

    State state = State::Zero(2 * (linkCount + 1));
    state(linkCount + 1) = 0.2;
    for (std::size_t rod = 0; rod < linkCount; ++rod) {
        state(1 + rod) = 0.2 - 0.1 * static_cast<double>(rod);
        state(linkCount + 2 + rod) = 0.3 + 0.1 * static_cast<double>(rod);
    }

    const double initialEnergy = totalEnergy(state, parameters);
    for (int step = 0; step < 1000; ++step) {
        rk4Step(state, 0.001, 0.0, parameters);
    }
    const double relativeError =
        std::abs(totalEnergy(state, parameters) - initialEnergy)
        / std::abs(initialEnergy);
    require(
        relativeError < 1.0e-7,
        "Energy drift is too high in the "
            + std::to_string(linkCount) + "-rod dynamics test.");
}

void testStabilization(std::size_t linkCount) {
    Parameters parameters;
    parameters.setLinkCount(linkCount);
    SensorNoise noise;
    Simulation simulation(parameters, noise);
    simulation.setControllerEnabled(true);

    for (int step = 0; step < 2400; ++step) {
        simulation.update(0.005);
        require(
            simulation.state().allFinite(),
            "Non-finite state in the " + std::to_string(linkCount) + "-rod test.");
    }

    const State& state = simulation.state();
    require(
        std::abs(state(0)) < 0.1,
        "Cart did not return to the track centre in the "
            + std::to_string(linkCount) + "-rod test.");
    require(
        state.segment(1, linkCount).cwiseAbs().maxCoeff() < 0.01,
        "A rod did not converge to upright in the "
            + std::to_string(linkCount) + "-rod test.");
    require(
        state.tail(linkCount + 1).cwiseAbs().maxCoeff() < 0.05,
        "Residual velocity is too high in the "
            + std::to_string(linkCount) + "-rod test.");

    std::cout << linkCount << " rod(s): x=" << state(0)
              << ", max |theta|=" << state.segment(1, linkCount).cwiseAbs().maxCoeff()
              << ", max |velocity|=" << state.tail(linkCount + 1).cwiseAbs().maxCoeff()
              << '\n';
}

double maxUprightAngle(const State& state, std::size_t linkCount) {
    double result = 0.0;
    for (std::size_t rod = 0; rod < linkCount; ++rod) {
        const double theta = state(1 + rod);
        result = std::max(
            result, std::abs(std::atan2(std::sin(theta), std::cos(theta))));
    }
    return result;
}

void testSwingUp(std::size_t linkCount) {
    Parameters parameters;
    parameters.setLinkCount(linkCount);
    SensorNoise noise;
    Simulation simulation(parameters, noise);
    simulation.resetHanging();

    for (int step = 0; step < 12000; ++step) {
        simulation.update(0.005);
        require(
            simulation.state().allFinite(),
            "Non-finite state in the " + std::to_string(linkCount)
                + "-rod swing-up test.");
    }

    const State& state = simulation.state();
    const double maxAngle = maxUprightAngle(state, linkCount);
    const double maxVelocity = state.tail(linkCount + 1).cwiseAbs().maxCoeff();
    std::cout << linkCount << " rod(s) swing-up: x=" << state(0)
              << ", max |theta|=" << maxAngle
              << ", max |velocity|=" << maxVelocity
              << ", mode=" << (simulation.isSwingingUp() ? "swing" : "balance")
              << '\n';

    require(!simulation.isSwingingUp(), "LQR did not take over after swing-up.");
    require(maxAngle < 0.02, "Swing-up did not reach the upright equilibrium.");
    require(maxVelocity < 0.1, "Swing-up finished with excessive velocity.");
    require(std::abs(state(0)) < 0.2, "Swing-up left the cart off centre.");
}

void testDisturbanceRecovery(std::size_t linkCount) {
    Parameters parameters;
    parameters.setLinkCount(linkCount);
    SensorNoise noise;
    Simulation simulation(parameters, noise);

    State disturbed = State::Zero(2 * (linkCount + 1));
    disturbed(0) = 0.3;
    disturbed.segment(1, linkCount).setConstant(0.3);
    disturbed.tail(linkCount).setConstant(0.2);
    simulation.reset(disturbed);

    for (int step = 0; step < 4000; ++step) {
        simulation.update(0.005);
        require(simulation.state().allFinite(), "Non-finite recovery state.");
    }

    const State& state = simulation.state();
    require(!simulation.isSwingingUp(), "Recovery did not return to LQR mode.");
    require(
        maxUprightAngle(state, linkCount) < 0.02,
        "Recovery did not restore the upright equilibrium.");
    require(
        state.tail(linkCount + 1).cwiseAbs().maxCoeff() < 0.1,
        "Recovery finished with excessive velocity.");
    require(std::abs(state(0)) < 0.2, "Recovery left the cart off centre.");
}

} // namespace

int main() {
    try {
        testSingleRodDynamicsRegression();
        for (std::size_t links = 1; links <= 3; ++links) {
            testEnergyConservation(links);
            testStabilization(links);
            testSwingUp(links);
            testDisturbanceRecovery(links);
        }
    } catch (const std::exception& error) {
        std::cerr << "Test failed: " << error.what() << '\n';
        return 1;
    }

    std::cout << "All dynamics and control tests passed.\n";
    return 0;
}
