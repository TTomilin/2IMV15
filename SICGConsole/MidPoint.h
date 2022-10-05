#include "Solver.h"
#include "System.h"


class MidPoint
{

public:
    MidPoint(Solver* solver);
    void simulation_step(System *system, double dt);

private:
    Solver* solver;

};