#pragma once

#include <Eigen/Dense>

#include <cstddef>
#include <vector>

// State layout: [x, theta_1 ... theta_n, x_dot, omega_1 ... omega_n].
// Every rod angle is measured from the upward vertical direction.
using State = Eigen::VectorXd;

struct Parameters {
    double cartMass = 1.0;
    std::vector<double> rodMasses = {0.2};
    std::vector<double> rodLengths = {0.6};
    double gravity = 9.82;

    double trackMin = -5.0;
    double trackMax = 5.0;
    double cartHalfWidth = 0.28;
    double restitution = 1.0;

    double jointDamping = 0.01;
    double cartFriction = 0.05;

    std::size_t linkCount() const {
        return rodLengths.size();
    }

    void setLinkCount(std::size_t count);
};

Eigen::MatrixXd massMatrix(const State& y, const Parameters& p);
State derivative(const State& y, double force, const Parameters& p);
double totalEnergy(const State& y, const Parameters& p);
