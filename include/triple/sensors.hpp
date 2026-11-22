#pragma once

#include "triple/model.hpp"
#include <random>

struct Measurement {
    double x = 0.0;
    double theta = 0.0;
    double cartVelocity = 0.0;
    double omega = 0.0;
};

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
    Measurement read(const Vec4& trueState);
private:
    std::mt19937 randomEngine_;
    std::normal_distribution<double> standardNormal_;
    double sample(double standardDeviation);
};