#include "triple/sensors.hpp"

SensorSuite::SensorSuite():
    randomEngine_(std::random_device{}()),
    standardNormal_(0.0, 1.0)
{
}

double SensorSuite::sample(double standardDeviation) {
    if (standardDeviation == 0.0) {
        return 0.0;
    }
    return standardDeviation * standardNormal_(randomEngine_);
}

Measurement SensorSuite::read(const Vec4& y) {
    Measurement result;

    result.x = 
        y(0) + sample(noise.xStdDev);
    result.theta = 
        y(1) + sample(noise.thetaStdDev);
    result.cartVelocity = 
        y(2) + sample(noise.cartVelocityStdDev);
    result.omega = 
        y(3) + sample(noise.omegaStdDev);

    return result;
}