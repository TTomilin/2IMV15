#include "stdafx.h"
#include "Implicit.h"
#include "MatX.h"


Implicit::Implicit(Solver* solver) {
	this->solver = solver;
}


void Implicit::simulation_step(System* system, double h)
{
	System::State start = system->get_state();
	vector<Particle*> particles = start.particles;
	vector<RigidBody*> bodies = start.bodies;
	vector<Force*> forces = *(system->get_forces());

	// Get the derivative
	vector<Solver::Vec3Tuple> derivative_p(particles.size());
	vector<RigidBody::State> derivative_b(bodies.size());
	solver->particleDerivative(derivative_p);
	solver->bodyDerivative(derivative_b);

	bool debug = false;

	int dims = 3;
	double epsilon = 1e-8;
	double step = 0.25; 

	// Do Derivative_*.force = (1/h I - f'(Y0))^-1 * derivative.force
	int size = (particles.size()) * dims; // 1 extra for the mouse particle
	double* right = new double[size];
	double* x0 = new double[size];
	double* vh = new double[size];
	double* xh = new double[size];
	double* v0 = new double[size];
	double* f0 = new double[size];
	MatX M = MatX(size);

	fill(xh, xh + size, 0.);
	fill(vh, vh + size, 0.);
	fill(v0, v0 + size, 0.);
	fill(f0, f0 + size, 0.);

	for (int i = 0; i < particles.size(); ++i) {
		Particle* particle = particles[i];
		int index = i * dims;

		Vec3 p_x0 = particle->m_Position;
		x0[index + 0] = p_x0[0];
		x0[index + 1] = p_x0[1];
		x0[index + 2] = p_x0[2];

		Vec3 p_v0 = derivative_p[i].vec1;
		Vec3 p_f0 = derivative_p[i].vec2;
		v0[index + 0] = p_v0[0];
		v0[index + 1] = p_v0[1];
		v0[index + 2] = p_v0[2];
		f0[index + 0] = p_f0[0];
		f0[index + 1] = p_f0[1];
		f0[index + 2] = p_f0[2];

		//Vec3 m = new Vec3(0., 0., 0.);
		//for (int i = 0; i < 3; i++)
		//{
		//	m[i] = particle->m_MassInv[i] == 0 ? 1000000. : 1. / particle->m_MassInv[i];
		//}

		Mat3* mat = new Mat3(
				Vec3(particle->m_MassInv[0] == 0 ? 1000000. : 1. / particle->m_MassInv[0],    0,    0),
				Vec3(0, particle->m_MassInv[1] == 0 ? 1000000. : 1. / particle->m_MassInv[1],    0),
				Vec3(0,       0, particle->m_MassInv[2] == 0 ? 1000000. : 1. / particle->m_MassInv[2])
		);
		

		M.addToSection(i, i, *mat);
		delete mat;
		// TODO Remove new vectors.
		

		//right[index + 0] = step * (f0[0] 
		//right[index + 1] = v0[1] + step * f0[1] * particle->m_MassInv;
		//right[index + 2] = v0[2] + step * f0[2] * particle->m_MassInv;
	}
	// TODO Add mouse particle if existing

	// Get Jacobian Matrix of forces
	if (debug) cout << "Getting Jx" << endl;
	MatX Jx = solver->calculateJacobianX(particles, forces);
	if (debug) cout << "Getting Jv" << endl;
	MatX Jv = solver->calculateJacobianV(particles, forces);
	//MatX Jx = MatX(size);
	//MatY Jv = MatX(size);

	// Solve
	if (debug) cout << "Calc left" << endl;
	MatX left = M.sub(Jv.mult(step)).sub(Jx.clone().mult(step * step));
	if (debug) cout << "Calc right 1" << endl;
	right = Jx.mult(step).mult(v0);
	if (debug) cout << "Calc right 2" << endl;
	for (int i = 0; i < particles.size(); ++i) {
		int index = i * dims;
		right[index + 0] = (right[index + 0] + f0[index + 0]) * step;
		right[index + 1] = (right[index + 1] + f0[index + 1]) * step;
		right[index + 2] = (right[index + 2] + f0[index + 2]) * step;
		if (debug) cout << "Right: " << right[index] << ", " << right[index + 1] << ", " << right[index + 2] << endl;
	}

	int steps = 0;
	if (debug) cout << "Starting ConjGrad" << endl;
	double gradient = ConjGrad(size, &left, vh, right, epsilon, &steps);
	if (debug) cout << "Finished ConjGrad" << endl;

	// Apply to particles
	for (int i = 0; i < particles.size(); ++i) {
		Particle* particle = particles[i];
		int index = i * dims;
		if (debug) cout << vh[index] << ", " << vh[index + 1] << ", " << vh[index + 2] << endl;
		if (particle->movable) {
			//particle->m_Position += Vec3(vh[index + 0], vh[index + 1], vh[index + 2]);
			particle->m_Velocity += Vec3(vh[index + 0], vh[index + 1], vh[index + 2]);
			particle->m_Position += step * particle->m_Velocity;
		}
	}

	// Bodies
	int size_b = bodies.size();
	for (int i = 0; i < size_b; i++) {
		if (bodies.at(i)->movable) {
			bodies.at(i)->setState(RigidBody::State(
				bodies.at(i)->x + derivative_b.at(i).x * step,
				bodies.at(i)->q + derivative_b.at(i).q * step,
				bodies.at(i)->P + derivative_b.at(i).P * step,
				bodies.at(i)->L + derivative_b.at(i).L * step
			));
		}
	}

	system->set_state(start);

	// NB: Solver is not called to apply this
}
