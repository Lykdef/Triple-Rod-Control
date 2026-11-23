#include "triple/simulation.hpp"

#include "triple/collision.hpp"
#include "triple/integrator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

Simulation::Simulation(const Parameters& parameters, const SensorNoise& noise)
    : parameters_(parameters),
      controller_(parameters_, dt_) {
    sensors_.noise = noise;
    reset();
}

void Simulation::reset() {
    State initialState = State::Zero(2 * (parameters_.linkCount() + 1));
    initialState.segment(1, parameters_.linkCount()).setConstant(0.05);
    reset(initialState);
}

void Simulation::resetHanging() {
    State initialState = State::Zero(2 * (parameters_.linkCount() + 1));
    constexpr double pi = 3.14159265358979323846;
    initialState.segment(1, parameters_.linkCount()).setConstant(pi);
    reset(initialState);
}

void Simulation::reset(const State& initialState) {
    const Eigen::Index expectedSize =
        static_cast<Eigen::Index>(2 * (parameters_.linkCount() + 1));
    if (initialState.size() != expectedSize) {
        throw std::invalid_argument("Initial state does not match the rod count.");
    }

    state_ = initialState;
    accumulator_ = 0.0;
    appliedForce_ = 0.0;
    controller_.reset(state_);
}

void Simulation::update(double frameTime) {
    if (!running_) {
        // Do not accumulate a backlog while the simulation is paused.
        accumulator_ = 0.0;
        return;
    }

    accumulator_ += frameTime;
    while (accumulator_ >= dt_) {
        double controlForce = manualForce_;
        if (controllerEnabled_) {
            controlForce = controller_.computeForce(sensors_.read(state_));
        }

        appliedForce_ = std::clamp(
            controlForce, -maxControlForce_, maxControlForce_);
        rk4Step(state_, dt_, appliedForce_, parameters_);
        solveWallCollision(state_, parameters_);
        accumulator_ -= dt_;
    }
}

const State& Simulation::state() const {
    return state_;
}

const Parameters& Simulation::parameters() const {
    return parameters_;
}

void Simulation::setLinkCount(std::size_t count) {
    if (parameters_.linkCount() == count) {
        return;
    }
    parameters_.setLinkCount(count);
    controller_ = AutomaticController(parameters_, dt_);
    reset();
}

bool Simulation::isRunning() const {
    return running_;
}

void Simulation::setRunning(bool running) {
    if (running_ == running) {
        return;
    }
    running_ = running;
    accumulator_ = 0.0;
}

bool Simulation::isControllerEnabled() const {
    return controllerEnabled_;
}

void Simulation::setControllerEnabled(bool enabled) {
    if (controllerEnabled_ != enabled && enabled) {
        controller_.reset(state_);
    }
    controllerEnabled_ = enabled;
}

bool Simulation::isSwingingUp() const {
    return controllerEnabled_ && controller_.isSwingingUp();
}

double Simulation::manualForce() const {
    return manualForce_;
}

void Simulation::setManualForce(double force) {
    manualForce_ = force;
}

double Simulation::appliedForce() const {
    return appliedForce_;
}
