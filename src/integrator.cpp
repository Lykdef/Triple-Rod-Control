#include "triple/integrator.hpp"

void rk4Step(State& y, double dt, double force, const Parameters& p) {
    const State k1 = derivative(y, force, p);
    const State k2 = derivative(y + 0.5 * dt * k1, force, p);
    const State k3 = derivative(y + 0.5 * dt * k2, force, p);
    const State k4 = derivative(y + dt * k3, force, p);

    y += (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}
