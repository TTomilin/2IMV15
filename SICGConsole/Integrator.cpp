#include "stdafx.h"
#include "Integrator.h"
#include "collisionDetector.h"


Integrator::Integrator(Solver* solver) {
    euler = Euler(solver);
    midpoint = MidPoint(solver);
    rungekutta = RungeKutta(solver);
    implicit = Implicit(solver);
}


Integrator::~Integrator() {}


void Integrator::simulation_step(System *system, int type, bool adaptive)
{
    if (adaptive && type != 4) {
        // Start state
        System::State start = system->get_state();

        // Get a sample upon doing a full stepsize
        execute_step(system, type, system->dt, system->dt, false);
        std::vector<Particle*> stateAfterFull = system->get_state().particles;
        system->set_state(start);

        // Get a sample upon doing a half stepsize twice
        execute_step(system, type, system->dt / 2., system->dt / 2., false);
        execute_step(system, type, system->dt / 2., system->dt / 2., false);
        std::vector<Particle*> stateAfterDoubleHalf = system->get_state().particles;
        system->set_state(start);

        // Calculate the error, namely the difference in the results
        double squareSum = 0;
        for (int i = 0; i < start.particles.size(); i++) {
            Particle a = *stateAfterFull.at(i);
            Particle b = *stateAfterDoubleHalf.at(i);
            squareSum += pow(b.m_Position[0] - a.m_Position[0], 2.);
            squareSum += pow(b.m_Position[1] - a.m_Position[1], 2.);
            squareSum += pow(b.m_Position[2] - a.m_Position[2], 2.);
            squareSum += pow(b.m_Velocity[0] - a.m_Velocity[0], 2.);
            squareSum += pow(b.m_Velocity[1] - a.m_Velocity[1], 2.);
            squareSum += pow(b.m_Velocity[2] - a.m_Velocity[2], 2.);
        }
        double error = sqrt(squareSum);

        // Adjust the stepsize
        if (error > 0) {
            switch (type) { // Different integrators have different error sizes
            case 1: // Euler O(h^2)
                system->dt *= pow(allowed_error / error, 1./2.);
                break;

            case 2: // Midpoint O(h^3)
                system->dt *= pow(allowed_error / error, 1./3.);
                break;

            case 3: // Range Kutta 4 O(h^5)
                system->dt *= pow(allowed_error / error, 1./5.);
                break;

            case 4: 
                // Implicit, don't do adaptive step size
                break;

            case 5:
                printf("\nNot yet implemented, terminating program");
                exit(0);
                break;

            default:
                printf("\r\tUnknown simulation type: %d, step: %6d", type, ++counter);
                exit(0);
                break;
            }
        }
    }
    else {
        system->dt = system->START_DT;
    }


    // Update timers.
    QueryPerformanceCounter((LARGE_INTEGER*) &cpu_end_time);
    time_simulation += system->dt;
    
    execute_step(system, type, system->dt, system->dt, true);

    collisions_enabled = new_collisions_enabled;
}


void Integrator::set_collisions(bool enable)
{
    new_collisions_enabled = enable;
}

bool Integrator::is_collisions_enabled()
{
    return new_collisions_enabled;
}

/**
 * Execute taking a step of type with the given dt
 */
void Integrator::execute_step(System *system, int type, double dt, double original_stepsize, bool print_counter)
{
    vector<Contact> contacts;
    System::State original = system->get_state();
    
    switch (type) {

    case 1:
        if (print_counter) printf("\r\tEuler simulation step: %6d stepsize %6.4f cpu time: %8.2f, simulation time: %8.5f", 
                                    ++counter, system->dt,(float)(cpu_end_time - cpu_start_time) / 1e6, time_simulation);
        euler.simulation_step(dt);
        break;

    case 2:
        if (print_counter) printf("\r\tMidPoint simulation step: %6d stepsize %6.4f cpu time: %8.2f, simulation time: %8.5f",
            ++counter, system->dt, (float)(cpu_end_time - cpu_start_time) / 1e6, time_simulation);
        midpoint.simulation_step(system, dt);
        break;

    case 3:
        if (print_counter) printf("\r\tRunge Kutta 4 simulation step:  %6d stepsize %6.4f cpu time: %8.2f, simulation time: %8.5f",
            ++counter, system->dt, (float)(cpu_end_time - cpu_start_time) / 1e6, time_simulation);
        rungekutta.simulation_step(system, dt);
        break;

    case 4:
        if (print_counter) printf("\r\tImplicit simulation step: %6d stepsize %6.4f", ++counter, system->dt);
        implicit.simulation_step(system, dt);
        break;

    case 5:
        if (print_counter) printf("\r\tVerlet simulation step:  %6d stepsize %6.4f cpu time: %8.2f, simulation time: %8.5f",
            ++counter, system->dt, (float)(cpu_end_time - cpu_start_time) / 1e6, time_simulation);
        printf("\nNot yet implemented, terminating program");
        exit(0);
        break;

    default:
        printf("\r\tUnknown simulation type: %d\n\tTerminating program!\n", type);
        exit(0);
        break;
    }

    if (collisions_enabled) {
        // Get contacts
        CollisionDetector coll_detector = CollisionDetector();

        // Check contacts
        bool had_collision = false;
        do {
            had_collision = false;

            // Find contacts
            coll_detector.find_contacts(System::State(*system->get_particles(), *system->get_bodies()), contacts);
            // Check contacts
            if (!contacts.size() == 0) {
                for (Contact contact : contacts) {
                    if (!contact.vf) {
                        // Inter-penetration
                        had_collision = true;
                        // Rollback simulation
                        system->set_state(original);
                        // printf("Rolling back from dt %.12f to %.12f\n", dt, (dt - dt / 2.0));
                        execute_step(system, type, (dt - dt / 2.0), original_stepsize, print_counter);
                        // Interrupt contacts processing
                        break;
                    }
                    else {
                        //printf("Check if collision happening\n");
                        if (coll_detector.colliding(contact)) {
                            // Surface contact
                            coll_detector.collision(contact);
                        }
                    }
                }
            }
            else {
                if (dt != original_stepsize) {
                    system->set_state(original);
                    // printf("Rolling forward from dt %.12f to %.12f\n", dt, (dt + dt / 2.0));
                    execute_step(system, type, (dt + dt / 2.0), original_stepsize, print_counter);
                }
            }
        } while (had_collision);

        // Clear memory
        original.particles.clear();
        original.bodies.clear();
    }
}

void Integrator::reset()
{
    QueryPerformanceCounter((LARGE_INTEGER*)&cpu_start_time);
    time_simulation = 0.f;
    counter = 0;
}
