#pragma once

#include "triple/lqr.hpp"
#include "triple/model.hpp"

// Multi-link reference swing-up and energy pumping with LQR handoff.
class AutomaticController {
public:
    AutomaticController(const Parameters& parameters, double sampleTime);

    double computeForce(const State& measurement);
    void reset(const State& initialState);

    bool isSwingingUp() const {
        return swingingUp_;
    }

private:
    double swingUpForce(const State& measurement) const;
    double trajectoryForce(const State& measurement) const;
    double uprightEnergy() const;

    Parameters parameters_;
    LqrController lqr_;
    double sampleTime_;
    double trajectoryTime_ = 0.0;
    bool useTrajectory_ = false;
    bool swingingUp_ = true;
};
