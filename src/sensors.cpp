#include "triple/sensors.hpp"

SensorSuite::SensorSuite()
    : randomEngine_(std::random_device{}()),
      standardNormal_(0.0, 1.0) {
}

double SensorSuite::sample(double standardDeviation) {
    if (standardDeviation == 0.0) {
        return 0.0;
    }
    return standardDeviation * standardNormal_(randomEngine_);
}

State SensorSuite::read(const State& y) {
    const std::size_t n = (y.size() - 2) / 2;
    State result = y;

    result(0) += sample(noise.xStdDev);
    result(n + 1) += sample(noise.cartVelocityStdDev);
    for (std::size_t rod = 0; rod < n; ++rod) {
        result(1 + rod) += sample(noise.thetaStdDev);
        result(n + 2 + rod) += sample(noise.omegaStdDev);
    }
    return result;
}
