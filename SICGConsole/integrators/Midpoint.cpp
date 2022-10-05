#include "stdafx.h"
#include "midpoint.h"


Vec3f midpoint::calculate(Vec3f position, float dt, float stepsize)
{
    // Perform Euler step update
    Vec3f step = Vec3f(stepsize, stepsize, stepsize);
    Vec3f update = dt * ((position + step) - (position - step)) / (2.0f * stepsize);

    // Evaluate at the midpoint
    Vec3f midpoint_position = (step + update) / 2.0f;
    float midpoint_dt = dt / 2.0f;

    update = midpoint_dt * ((midpoint_position + step) - (midpoint_position - step)) / (2.0f * stepsize);
    return update;
}
