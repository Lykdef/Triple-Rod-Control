#pragma once

#include "triple/model.hpp"

// Linear-quadratic regulator for the upright equilibrium.
// Gains are computed from the same nonlinear model used by the simulation.
class LqrController {
public:
    LqrController(const Parameters& parameters, double sampleTime);

    double computeForce(const State& measurement) const;

private:
    Eigen::MatrixXd gain_;
};
