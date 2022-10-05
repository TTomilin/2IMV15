#include "stdafx.h"
#include "Midpoint.h"
#include "RigidBody.h"


MidPoint::MidPoint(Solver* solver) {
    this->solver = solver;
}


void MidPoint::simulation_step(System *system, double dt)
{
    System::State start = system->get_state();
    vector<Particle*> particles = start.particles;
    vector<RigidBody*> bodies = start.bodies;

    // Get to the midpoint
    solver->simulation_step(dt / 2.);

    // Get the derivative at the midpoint
    vector<Solver::Vec3Tuple> midDerivative_p(particles.size());
    vector<RigidBody::State> midDerivative_b(bodies.size());
    solver->particleDerivative(midDerivative_p);
    solver->bodyDerivative(midDerivative_b);
    
    // Update state from start position using derivative at mid position
    system->set_state(System::State(particles, bodies));
    solver->applyDerivative(midDerivative_p, midDerivative_b, dt);
}
