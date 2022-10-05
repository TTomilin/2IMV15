#pragma once
#include "include/gfx/vec3.h"
class Base
{

public:
    
    // Pure vriutal function to provide framework interface
    virtual Vec3f calculate(Vec3f position, float dt, float stepsize) = 0;

};

