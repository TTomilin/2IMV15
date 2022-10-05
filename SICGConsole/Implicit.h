#pragma once
#include "Solver.h"


class Implicit
{

public:
    Implicit(Solver* solver);
    void simulation_step(System *system, double dt);

private:
    Solver* solver;

};