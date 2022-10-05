#include "Solver.h"
#include "System.h"


class RungeKutta
{

public:
    RungeKutta(Solver* solver);
    void simulation_step(System *system, double dt);

private:
    Solver* solver;

};