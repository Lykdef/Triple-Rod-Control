#pragma once

#include "triple/model.hpp"

void rk4Step(Vec4& y, double dt, double force, const Parameters& p);