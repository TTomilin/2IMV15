#include "include/gfx/vec3.h"
#include "Solver.h"

#include "Euler.h"
#include "MidPoint.h"
#include "RungeKutta.h"
#include "Implicit.h"
#include "System.h"


class Integrator
{

private:
    Euler euler = nullptr;
    MidPoint midpoint = nullptr;
    RungeKutta rungekutta = nullptr;
    Implicit implicit = nullptr;

public:
    Integrator(Solver* solver);
    virtual ~Integrator();
    
    void simulation_step(System *system, int type, bool adaptive);
    void reset();

    int counter = 0;
    double allowed_error = 0.001;
    void reset_timings();

    void set_collisions(bool enable);
    bool is_collisions_enabled();

private:
    void execute_step(System* system, int type, double stepsize, double original_stepsize, bool print);

    // Timings
    LARGE_INTEGER* cpu_start_time;
    LARGE_INTEGER* cpu_end_time;
    int frequency = 1;
    float time_simulation = 0.f;

    bool new_collisions_enabled = true;
    bool collisions_enabled = true;

};

 