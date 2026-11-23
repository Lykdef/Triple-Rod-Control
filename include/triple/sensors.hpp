#pragma once

#include "triple/model.hpp"

#include <random>

struct SensorNoise {
    double xStdDev = 0.0;
    double thetaStdDev = 0.0;
    double cartVelocityStdDev = 0.0;
    double omegaStdDev = 0.0;
};

class SensorSuite {
public:
    SensorNoise noise;

    SensorSuite();
    State read(const State& trueState);

private:
    std::mt19937 randomEngine_;
    std::normal_distribution<double> standardNormal_;

    double sample(double standardDeviation);
};
