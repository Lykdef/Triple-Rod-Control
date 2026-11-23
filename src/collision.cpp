#include "triple/collision.hpp"

void solveWallCollision(State& y, const Parameters& p) {
    const std::size_t n = p.linkCount();
    const double leftLimit = p.trackMin + p.cartHalfWidth;
    const double rightLimit = p.trackMax - p.cartHalfWidth;
    const double cartVelocity = y(n + 1);

    bool collided = false;
    if (y(0) < leftLimit) {
        y(0) = leftLimit;
        collided = cartVelocity < 0.0;
    } else if (y(0) > rightLimit) {
        y(0) = rightLimit;
        collided = cartVelocity > 0.0;
    }

    if (!collided) {
        return;
    }

    // A wall impulse acts only on the cart coordinate. The inverse mass
    // matrix determines how that impulse changes every joint velocity.
    Eigen::VectorXd cartDirection = Eigen::VectorXd::Zero(n + 1);
    cartDirection(0) = 1.0;
    const Eigen::VectorXd velocityPerImpulse =
        massMatrix(y, p).ldlt().solve(cartDirection);
    const double impulse = -(1.0 + p.restitution) * cartVelocity
        / velocityPerImpulse(0);

    y.tail(n + 1) += velocityPerImpulse * impulse;
}
