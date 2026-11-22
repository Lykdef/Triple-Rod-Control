#include "triple/model.hpp"
#include <cmath>

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

    const double comVelocityX = cartVelocity + a * omega * std::cos(theta);
    const double comVelocityY = - a * omega * std::sin(theta);
    const double cartKinetic = 0.5 * M * cartVelocity * cartVelocity;
    const double rodTranslationalKinetic = 
        0.5 * m * (comVelocityX * comVelocityX + comVelocityY * comVelocityY);
    const double rodRotationalKinetic = 0.5 * I * omega * omega;
    const double potential = m * g * a * std::cos(theta);

    return cartKinetic + rodTranslationalKinetic + rodRotationalKinetic + potential;
}