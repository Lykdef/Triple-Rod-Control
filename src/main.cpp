#include <iostream>
#include <iomanip>
#include <Eigen/Dense>

using Vec4 = Eigen::Vector4d;

struct Parameters {
    double cartMass = 1.0;
    double rodMass = 0.2;
    double rodLength = 0.6;
    double gravity = 9.81;
};

// Function to compute the derivative of the state vector
Vec4 derivative(const Vec4& y, double force, const Parameters& p) {
    const double theta = y(1);
    const double omega = y(3);

    const double M = p.cartMass;
    const double m = p.rodMass;
    const double L = p.rodLength;
    const double g = p.gravity;

    const double a = L * 0.5;
    const double I = m * L * L / 12.0;
    const double J = I + m * a * a;
    
    const double c = std::cos(theta);
    const double s = std::sin(theta);

    const double A = M + m;
    const double B = m * a * c;
    const double R = force + m * a * omega * omega * s;
    const double G = m * g * a * s;

    const double determinant = A * J - B * B;
    
    const double cartAcceleration = (J * R - B * G) / determinant;
    const double angularAcceleration = (-B * R + A * G) / determinant;

    Vec4 dy;
    dy << y(2), y(3), cartAcceleration, angularAcceleration;
    return dy;
}
// Function to compute the total energy of the system
double totalEnergy(const Vec4& y, const Parameters& p) {
    const double theta = y(1);
    const double cartVelocity = y(2);
    const double omega = y(3);

    const double M = p.cartMass;
    const double m = p.rodMass;
    const double L = p.rodLength;
    const double g = p.gravity;

    const double a = L * 0.5;
    const double I = m * L * L / 12.0;

    const double comVelocityX = cartVelocity - a * omega * std::cos(theta);
    const double comVelocityY = - a * omega * std::sin(theta);
    const double cartKinetic = 0.5 * M * cartVelocity * cartVelocity;
    const double rodTranslationalKinetic = 
        0.5 * m * (comVelocityX * comVelocityX + comVelocityY * comVelocityY);
    const double rodRotationalKinetic = 0.5 * I * omega * omega;
    const double potential = m * g * a * std::cos(theta);

    return cartKinetic + rodTranslationalKinetic + rodRotationalKinetic + potential;
}
// Function to perform a single RK4 step
void rk4Step(Vec4& y, double dt, double force, const Parameters& p) {
    Vec4 k1 = derivative(y, force, p);
    Vec4 k2 = derivative(y + 0.5 * dt * k1, force, p);
    Vec4 k3 = derivative(y + 0.5 * dt * k2, force, p);
    Vec4 k4 = derivative(y + dt * k3, force, p);

    y += (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}
//
int main() {
    const Parameters p;

    Vec4 y; // State vector: [x, theta, x_dot, theta_dot]
    y << 0.0, 0.15, 0.0, 0.0;

    const double dt = 0.001; // Time step
    const double simulationTime = 5.0; // Total simulation time
    const int steps = static_cast<int>(simulationTime / dt);

    const double force = 0.0; // No external force applied
    const double initialEnergy = totalEnergy(y, p);

    std::cout << std::fixed << std::setprecision(9);
    std::cout << "Initial Energy: " << initialEnergy << std::endl;

    for (int step = 0; step < steps; ++step) {
        if (step % 100 == 0) { // Print every 1000 steps
            const double time = step * dt;
            const double energyDrift = totalEnergy(y, p) - initialEnergy;
            std::cout << "t: " << time << " s, "
                      << "x: " << y(0) << ", "
                      << "theta: " << y(1) << ", "
                      << "x_dot: " << y(2) << ", "
                      << "theta_dot: " << y(3) << ", "
                      << "Energy Drift: " << energyDrift
                      << std::endl;
        }
        rk4Step(y, dt, force, p);
    }
    return 0;
}