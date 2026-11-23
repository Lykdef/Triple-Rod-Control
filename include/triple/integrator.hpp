#pragma once

#include "triple/model.hpp"

void rk4Step(State& y, double dt, double force, const Parameters& p);
