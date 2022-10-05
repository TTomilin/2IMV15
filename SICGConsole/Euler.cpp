#include "stdafx.h"
#include "Euler.h"


Euler::Euler(Solver* solver) {
    this->solver = solver;
}


void Euler::simulation_step(double dt)
{   
    solver->simulation_step(dt);
}
