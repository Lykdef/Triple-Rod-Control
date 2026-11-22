#include "triple/simulation.hpp"

#include "triple/integrator.hpp"
#include "triple/collision.hpp"

#include <algorithm>
#include <cmath>

Simulation::Simulation(
    const Parameters& parameters,
    const SensorNoise& noise)
    : parameters_(parameters),
      positionPid_(1.0, 0.0, 2.0, 10.0),
      anglePid_(20.0, 0.0, 4.0, 10.0) {
    sensors_.noise = noise;

    reset();
}

void Simulation::reset() {
    state_ << 0.0, 0.05, 0.0, 0.0;

    accumulator_ = 0.0;
    appliedForce_ = 0.0;

    positionPid_.reset();
    anglePid_.reset();
}

void Simulation::update(double frameTime) {
    if (!running_) {
        // 暂停期间不积累时间。
        // 否则恢复运行后可能一次执行大量物理步。
        accumulator_ = 0.0;
        return;
    }

    accumulator_ += frameTime;

    while (accumulator_ >= dt_) {
        double controlForce = manualForce_;

        if (controllerEnabled_) {
            const Measurement measured =
                sensors_.read(state_);

            double angleError = -measured.theta;
            angleError = std::atan2(
                std::sin(angleError),
                std::cos(angleError));

            const double angleErrorRate =
                -measured.omega;

            const double angleOutput =
                anglePid_.update(
                    angleError,
                    angleErrorRate,
                    dt_);

            controlForce = -angleOutput;
        }

        appliedForce_ = std::clamp(
            controlForce,
            -maxControlForce_,
            maxControlForce_);

        rk4Step(state_, dt_, appliedForce_, parameters_);
        solveWallCollision(state_, parameters_);

        accumulator_ -= dt_;
    }
}

const Vec4& Simulation::state() const {
    return state_;
}

const Parameters& Simulation::parameters() const {
    return parameters_;
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
    if (controllerEnabled_ == enabled) {
        return;
    }
    controllerEnabled_ = enabled;

    positionPid_.reset();
    anglePid_.reset();
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