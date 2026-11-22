#include "triple/collision.hpp"
#include <cmath>

// Function to handle wall collisions and update the state vector accordingly
void solveWallCollision(Vec4& y, const Parameters& p) {
    const double leftLimit =
        p.trackMin + p.cartHalfWidth;

    const double rightLimit =
        p.trackMax - p.cartHalfWidth;

    bool collided = false;

    // 左墙
    if (y(0) < leftLimit) {
        y(0) = leftLimit;

        // 只有仍在向墙内运动时才处理碰撞
        if (y(2) < 0.0) {
            collided = true;
        }
    }
    // 右墙
    else if (y(0) > rightLimit) {
        y(0) = rightLimit;

        if (y(2) > 0.0) {
            collided = true;
        }
    }

    if (!collided) {
        return;
    }

    const double theta = y(1);
    const double oldCartVelocity = y(2);
    const double oldOmega = y(3);

    const double M = p.cartMass;
    const double m = p.rodMass;
    const double L = p.rodLength;

    const double a = L * 0.5;
    const double I = m * L * L / 12.0;

    const double A = M + m;
    const double B = m * a * std::cos(theta);
    const double J = I + m * a * a;
    const double determinant = A * J - B * B;

    const double e = p.restitution;

    // 墙面施加在滑块上的法向冲量
    const double impulse =
        -(1.0 + e)
        * determinant / J
        * oldCartVelocity;

    // 由 M * delta_v = [impulse, 0] 更新速度
    y(2) =
        oldCartVelocity
        + J / determinant * impulse;

    y(3) =
        oldOmega
        - B / determinant * impulse;
}
