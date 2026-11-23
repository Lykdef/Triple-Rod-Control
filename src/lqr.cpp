#include "triple/lqr.hpp"

#include <cmath>
#include <stdexcept>

namespace {

struct LinearModel {
    Eigen::MatrixXd a;
    Eigen::MatrixXd b;
};

LinearModel linearizeAtUpright(const Parameters& parameters) {
    const std::size_t stateSize = 2 * (parameters.linkCount() + 1);
    const State equilibrium = State::Zero(stateSize);
    const double epsilon = 1.0e-6;

    LinearModel model{
        Eigen::MatrixXd::Zero(stateSize, stateSize),
        Eigen::MatrixXd::Zero(stateSize, 1)
    };

    for (std::size_t column = 0; column < stateSize; ++column) {
        State plus = equilibrium;
        State minus = equilibrium;
        plus(column) += epsilon;
        minus(column) -= epsilon;
        model.a.col(column) =
            (derivative(plus, 0.0, parameters)
            - derivative(minus, 0.0, parameters))
            / (2.0 * epsilon);
    }

    model.b =
        (derivative(equilibrium, epsilon, parameters)
        - derivative(equilibrium, -epsilon, parameters))
        / (2.0 * epsilon);
    return model;
}

Eigen::MatrixXd solveGain(
    const Eigen::MatrixXd& continuousA,
    const Eigen::MatrixXd& continuousB,
    double sampleTime,
    const Eigen::MatrixXd& q,
    double r) {
    const std::size_t stateSize = continuousA.rows();
    const Eigen::MatrixXd identity =
        Eigen::MatrixXd::Identity(stateSize, stateSize);
    const Eigen::MatrixXd aSquared = continuousA * continuousA;

    // A short Taylor expansion is accurate at the fixed 1 ms physics step.
    const Eigen::MatrixXd a = identity
        + sampleTime * continuousA
        + 0.5 * sampleTime * sampleTime * aSquared;
    const Eigen::MatrixXd b =
        (sampleTime * identity
        + 0.5 * sampleTime * sampleTime * continuousA
        + sampleTime * sampleTime * sampleTime * aSquared / 6.0)
        * continuousB;

    Eigen::MatrixXd p = q;
    Eigen::MatrixXd gain;

    // Fixed-point iteration of the discrete algebraic Riccati equation.
    // The matrices are at most 8x8 for the three-rod configuration.
    for (int iteration = 0; iteration < 100000; ++iteration) {
        Eigen::MatrixXd denominator = b.transpose() * p * b;
        denominator(0, 0) += r;
        gain = denominator.ldlt().solve(b.transpose() * p * a);
        const Eigen::MatrixXd next =
            q + a.transpose() * p * (a - b * gain);
        const Eigen::MatrixXd symmetricNext = 0.5 * (next + next.transpose());

        const double change = (symmetricNext - p).cwiseAbs().maxCoeff();
        p = symmetricNext;
        if (change < 1.0e-10 * (1.0 + p.cwiseAbs().maxCoeff())) {
            return gain;
        }
    }

    throw std::runtime_error("LQR Riccati iteration did not converge.");
}

} // namespace

LqrController::LqrController(
    const Parameters& parameters,
    double sampleTime) {
    const std::size_t n = parameters.linkCount();
    const std::size_t stateSize = 2 * (n + 1);
    const LinearModel model = linearizeAtUpright(parameters);

    Eigen::VectorXd diagonal = Eigen::VectorXd::Zero(stateSize);
    diagonal(0) = 1.0;
    diagonal.segment(1, n).setConstant(30.0);
    diagonal(n + 1) = 0.5;
    diagonal.tail(n).setConstant(2.0);

    const Eigen::MatrixXd q = diagonal.asDiagonal();
    gain_ = solveGain(model.a, model.b, sampleTime, q, 0.1);
}

double LqrController::computeForce(const State& measurement) const {
    State error = measurement;
    const std::size_t n = (measurement.size() - 2) / 2;

    // LQR uses the shortest angular error around the upright target.
    for (std::size_t rod = 0; rod < n; ++rod) {
        const double theta = measurement(1 + rod);
        error(1 + rod) = std::atan2(std::sin(theta), std::cos(theta));
    }

    return (-gain_ * error)(0, 0);
}
