#include "Solver.h"


class Euler
{

public:
    Euler(Solver* solver);
    void simulation_step(double dt);


private:
    Solver* solver;
};

