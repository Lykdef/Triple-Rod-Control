#pragma once

#include <Eigen/Dense>

using Vec4 = Eigen::Vector4d;

struct Parameters {
    double cartMass = 1.0;
    double rodMass = 0.2;
    double rodLength = 0.6;
    double gravity = 9.82;

    double trackMin = -5;
    double trackMax = 5;
    double cartHalfWidth = 0.28;
    double restitution = 1.0;
};

Vec4 derivative(const Vec4& y, double force, const Parameters& p);
double totalEnergy(const Vec4& y, const Parameters& p);