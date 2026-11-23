#pragma once

#include "triple/automatic_controller.hpp"
#include "triple/model.hpp"
#include "triple/sensors.hpp"

#include <cstddef>

class Simulation {
public:
    Simulation(const Parameters& parameters, const SensorNoise& noise);

    void update(double frameTime);
    void reset();
    void resetHanging();
    void reset(const State& initialState);

    const State& state() const;
    const Parameters& parameters() const;
    void setLinkCount(std::size_t count);

    bool isRunning() const;
    void setRunning(bool running);

    bool isControllerEnabled() const;
    void setControllerEnabled(bool enabled);
    bool isSwingingUp() const;

    double manualForce() const;
    void setManualForce(double force);
    double appliedForce() const;

private:
    Parameters parameters_;
    State state_;
    SensorSuite sensors_;

    double dt_ = 0.001;
    AutomaticController controller_;
    double accumulator_ = 0.0;

    bool running_ = true;
    bool controllerEnabled_ = true;

    double manualForce_ = 0.0;
    double appliedForce_ = 0.0;
    double maxControlForce_ = 60.0;
};
