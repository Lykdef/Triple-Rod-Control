#include "triple/integrator.hpp"

// Function to perform a single RK4 step
void rk4Step(Vec4& y, double dt, double force, const Parameters& p) {
    Vec4 k1 = derivative(y, force, p);
    Vec4 k2 = derivative(y + 0.5 * dt * k1, force, p);
    Vec4 k3 = derivative(y + 0.5 * dt * k2, force, p);
    Vec4 k4 = derivative(y + dt * k3, force, p);

    y += (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}
