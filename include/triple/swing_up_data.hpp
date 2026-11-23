#pragma once

#include <cstddef>
#include <vector>

struct SwingUpTrajectory {
    std::size_t stateSize;
    const std::vector<double>& forces;
    const std::vector<double>& states;
    const std::vector<double>& feedbackGains;
};

// Precomputed 50 Hz trajectories for the default multi-rod models.
const SwingUpTrajectory* swingUpTrajectory(std::size_t linkCount);
constexpr double swingUpTimeStep = 0.02;
