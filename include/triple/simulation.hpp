#pragma once

#include "triple/model.hpp"
#include "triple/sensors.hpp"
#include "triple/pid.hpp"

class Simulation {
public:
    Simulation(
        const Parameters& parameters,
        const SensorNoise& noise);


    void update(double frameTime);

    void reset();

    const Vec4& state() const;
    const Parameters& parameters() const;

    bool isRunning() const;
    void setRunning(bool running);

    bool isControllerEnabled() const;
    void setControllerEnabled(bool enabled);

    double manualForce() const;
    void setManualForce(double force);

    double appliedForce() const;

private:
    Parameters parameters_;
    Vec4 state_ = Vec4::Zero();

    SensorSuite sensors_;

    PID positionPid_;
    PID anglePid_;

    double dt_ = 0.001;
    double accumulator_ = 0.0;

    bool running_ = true;
    bool controllerEnabled_ = false;

    double manualForce_ = 0.0;
    double appliedForce_ = 0.0;

    double maxControlForce_ = 30.0;
};