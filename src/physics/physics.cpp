#include "../../include/physics/physics.hpp"
#include <cmath>

namespace physics{
    const float PIXEL_PER_METER = 200;
    const float FIXED_DELTA_TIME = 1.f / 60.f;
    const int SUB_STEPS = 5;
    const float SUB_DELTA_TIME = FIXED_DELTA_TIME / SUB_STEPS;
    const float g = 9.8f;
    const float pi = 2 * std::acos(0.0f);
    const float frictionCoefficient = 0.2f;
    const float dragCoefficient = 0.47f;
    const float airDensity = 1.225f;
}