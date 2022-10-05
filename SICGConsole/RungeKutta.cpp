#include "stdafx.h"
#include "RungeKutta.h"
#include "RigidBody.h"


RungeKutta::RungeKutta(Solver* solver) {
    this->solver = solver;
}


void RungeKutta::simulation_step(System *system, double dt)
{
    // Start values
    System::State start = system->get_state();
    int size_p = start.particles.size();
    int size_b = start.bodies.size();
    vector<Solver::Vec3Tuple> k1_p(size_p);
    vector<RigidBody::State> k1_b(size_b);
    solver->particleDerivative(k1_p);
    solver->bodyDerivative(k1_b);

    // Use half of k1 (from start) to get k2
    solver->applyDerivative(k1_p, k1_b, dt / 2.);
    vector<Solver::Vec3Tuple> k2_p(size_p);
    vector<RigidBody::State> k2_b(size_b);
    solver->particleDerivative(k2_p);
    solver->bodyDerivative(k2_b);

    // Use half of k2 (from start) to get k3    
    system->set_state(start);
    solver->applyDerivative(k2_p, k2_b, dt / 2.);
    vector<Solver::Vec3Tuple> k3_p(size_p);
    vector<RigidBody::State> k3_b(size_b);
    solver->particleDerivative(k3_p);
    solver->bodyDerivative(k3_b);

    // Use k3 (from start) to get k4    
    system->set_state(start);
    solver->applyDerivative(k3_p, k3_b, dt);
    vector<Solver::Vec3Tuple> k4_p(size_p);
    vector<RigidBody::State> k4_b(size_b);
    solver->particleDerivative(k4_p);
    solver->bodyDerivative(k4_b);

    // Get the final position using all
    system->set_state(start);
    solver->applyDerivative(k1_p, k1_b, dt / 6.);
    solver->applyDerivative(k2_p, k2_b, dt / 3.);
    solver->applyDerivative(k3_p, k3_b, dt / 3.);
    solver->applyDerivative(k4_p, k4_b, dt / 6.);
}

