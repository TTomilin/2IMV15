#include "Particle.h"
#include "Constraint.h"
#include "Solver.h"
#include "JWJTransposed.h"
#include <assert.h>
#include "Mouse.h"
#include <Eigen/Dense>
#include <Eigen/IterativeLinearSolvers>
#include <cstdlib>
#include <ctime>

#define DIMS		3
#define THRESHOLD	0.1

using namespace Eigen;

Solver::Solver(const vector<Particle*>& particles, const vector<RigidBody*>& bodies,
	const vector<Force*>& forces, const vector<Force*>& constraints, const double ks,
	const double kd, const double epsilon, Mouse& mouse) :
	m_Particles(particles), m_Bodies(bodies),
	m_Forces(forces), m_Constraints(constraints),
	m_ks(ks), m_kd(kd), m_epsilon(epsilon), m_mouse(mouse) {}

void Solver::simulation_step(double dt) {
	vector<Vec3Tuple> forceTuples(m_Particles.size());
	vector<RigidBody::State> bodyStates(m_Bodies.size());
	particleDerivative(forceTuples);
	bodyDerivative(bodyStates);
	if (DEBUG) printf("\n\nSolver after derivative");
	applyDerivative(forceTuples, bodyStates, dt);
}

void Solver::applyDerivative(vector<Vec3Tuple> derivatives, vector<RigidBody::State> body_derivatives, double dt) {
	// Particles
	int i, size_p = m_Particles.size();
	for (i = 0; i < size_p; i++) {
		if (m_Particles[i]->movable) {
			m_Particles[i]->m_Position += derivatives[i].vec1 *= dt;
			m_Particles[i]->m_Velocity += derivatives[i].vec2 *= dt;
			if (DEBUG) printf("\nParticle %d. New position (%.6f, %.6f, %.6f). New velocities (%.6f, %.6f, %.6f)", i, m_Particles[i]->m_Position[0], m_Particles[i]->m_Position[1], m_Particles[i]->m_Position[2], m_Particles[i]->m_Velocity[0], m_Particles[i]->m_Velocity[1], m_Particles[i]->m_Velocity[2]);
		}
	}
	// Bodies
	int size_b = m_Bodies.size();
	for (i = 0; i < size_b; i++) {
		if (m_Bodies.at(i)->movable) {
			m_Bodies.at(i)->setState(RigidBody::State(
				m_Bodies.at(i)->x + body_derivatives.at(i).x * dt,
				m_Bodies.at(i)->q + body_derivatives.at(i).q * dt,
				m_Bodies.at(i)->P + body_derivatives.at(i).P * dt,
				m_Bodies.at(i)->L + body_derivatives.at(i).L * dt
			));
		}
	}
}

void Solver::particleDerivative(vector<Vec3Tuple>& forceTuples) {
	clearForces(m_Particles);
	calculateForces(m_Particles, m_Forces);
	if (m_mouse.getMouseSpring() != nullptr) {
		calculateForce(m_Particles, m_mouse.getMouseSpring());
	}
	calculateForces(m_Particles, m_Constraints);
	constrainForces();
	applyNewVelocities(forceTuples);
}

void Solver::bodyDerivative(vector<RigidBody::State>& bodyState)
{
	int size = m_Bodies.size();
	for (int i = 0; i < size; i++) {
		bodyState[i] = m_Bodies[i]->getDerivativeState();
	}
}

void Solver::clearForces(const vector<Particle*>& particles) {
	for (int i = 0; i < particles.size(); ++i) {
		particles[i]->m_AccumulatedForce = Vec3(0, 0, 0);
	}
}

void Solver::calculateForces(const vector<Particle*>& particles, const vector<Force*>& forces) {
	for (int i = 0; i < forces.size(); ++i) {
		calculateForce(particles, forces[i]);
	}
}

void Solver::calculateForce(const vector<Particle*>& particles, Force* force) {
	force->apply(particles);
}

MatX Solver::calculateJacobianX(const vector<Particle*>& particles, const vector<Force*>& forces) {
	MatX Jx = MatX(particles.size() * DIMS);
	for (Force* force : forces) {
		force->addToJx(Jx);
	}
	return Jx;
}

MatX Solver::calculateJacobianV(const vector<Particle*>& particles, const vector<Force*>& forces) {
	MatX Jv = MatX(particles.size() * DIMS);
	for (Force* force : forces) {
		force->addToJv(Jv);
	}
	return Jv;
}

/**
 * Section F6; paragraph 6.3; formula 11.
 */ 
void Solver::constrainForces() {
	if (m_Constraints.empty()) {
		return;
	}

	int dimensions = DIMS;
	const int n_particles = m_Particles.size();
	const int n_constraints = m_Constraints.size();
	const int n_particle_values = n_particles * dimensions;

	VectorXf QDot = VectorXf::Zero(n_particle_values);
	VectorXf Q = VectorXf::Zero(n_particle_values);
	MatrixXf W = MatrixXf::Zero(n_particle_values, n_particle_values);
	VectorXf C = VectorXf::Zero(n_constraints);
	VectorXf CDot = VectorXf::Zero(n_constraints);
	MatrixXf J = MatrixXf::Zero(n_constraints, n_particle_values);
	MatrixXf JT = MatrixXf::Zero(n_particle_values, n_constraints);
	MatrixXf JDot = MatrixXf::Zero(n_constraints, n_particle_values);

	for (int i = 0; i < n_particle_values; i += dimensions) {
		Particle* p = m_Particles[i / dimensions];
		for (int d = 0; d < dimensions; d++) {
			W(i + d, i + d) = p->m_MassInv[d];		// W is the inverse of mass M
			Q[i + d] = p->m_AccumulatedForce[d];	// Q contains the forces of the particles
			QDot[i + d] = p->m_Velocity[d];			// QDot contains the velocity of the particles
			//printf("\nQ[%d] (%.6f)", i + d, Q[i + d]);
			//printf("\nQDot[%d] (%.6f)", i + d, QDot[i + d]);
		}
	}

	printMatrix("W", n_particle_values, n_particle_values, W);
	printArray("Q", n_particle_values, Q);
	printArray("QDot", n_particle_values, QDot);

	// Compute the values for each constraint
	for (int i = 0; i < n_constraints; i++) {
		Constraint* constraint = (static_cast<Constraint*> (m_Constraints[i]));

		C[i] = constraint->getC();
		CDot[i] = constraint->getCDot();
		vector<Vector3f> j = constraint->getJ();
		vector<Vector3f> jd = constraint->getJDot();
		vector<Particle*> affected_particles = constraint->getParticles();

		// Fill the matrices at the correct particle positions
		for (int k = 0; k < affected_particles.size(); k++) {
			int p_index = affected_particles[k]->m_ID * dimensions;
			for (int d = 0; d < dimensions; d++) {
				JDot(i, p_index + d) = jd[k][d];
				J(i, p_index + d) = j[k][d];
				JT(p_index + d, i) = j[k][d];
			}
		}
	}

	printArray("C", n_constraints, C);
	printArray("CDot", n_constraints, CDot);

	printMatrix("J", n_constraints, n_particle_values, J);
	printMatrix("JDot", n_constraints, n_particle_values, JDot);
	printMatrix("Jt", n_particle_values, n_constraints, JT);

	MatrixXf JW = J * W;
	MatrixXf JWJT = JW * JT;
	VectorXf JDotQDot = JDot * QDot;
	VectorXf JWQ = JW * Q;

	printMatrix("JW", n_constraints, n_particle_values, JW);
	printMatrix("JWJT", n_constraints, n_constraints, JWJT);
	printArray("JDotQDot", n_constraints, JDotQDot);
	printArray("JWQ", n_constraints, JWQ);

	// Compute the right hand side of the formula
	VectorXf rhs = -JDotQDot - JWQ - m_ks * C - m_kd * CDot;
	printArray("rhs", n_constraints, rhs);

	// Initialize CG method
	ConjugateGradient<MatrixXf, Lower | Upper> cg;

	// Compute lambda
	cg.compute(JWJT);
	VectorXf lambda = cg.solve(rhs);
	printArray("lambda", n_constraints, lambda);

	VectorXf Qhat = J.transpose() * lambda;
	for (int i = 0; i < n_particles; i++) {
		Particle* p = m_Particles[i];
		int index = dimensions * i;
		for (int d = 0; d < dimensions; d++) {
			p->m_AccumulatedForce[d] += Qhat[index + d];
		}
	}
	printArray("Qhat", n_particle_values, Qhat);
}

void Solver::applyNewVelocities(vector<Vec3Tuple>& forceTuples) {
	if (DEBUG) printf("\n\nApplying new velocities");
	for (int i = 0; i < forceTuples.size(); ++i) {
		Particle* particle = m_Particles[i];
		forceTuples[i].vec1 = particle->m_Velocity;
		forceTuples[i].vec2 = Vec3(0., 0., 0.);
		for (size_t j = 0; j < 3; j++)
		{
			forceTuples[i].vec2[j] = particle->m_AccumulatedForce[j] * particle->m_MassInv[j];
		}
		
		Vec3 velocity = forceTuples[i].vec2; //particle->m_AccumulatedForce * particle->m_MassInv;
		if (DEBUG) {
				printf("\nParticle %d:", i);
				printf("\n\tInit: (% .6f, % .6f, % .6f)", particle->m_Velocity[0], particle->m_Velocity[1], particle->m_Velocity[2]);
				printf("\n\tNew : (% .6f, % .6f, % .6f)\n", velocity[0], velocity[1], velocity[2]);
			}
	}
}

void Solver::assertArray(double* array, int size) {
	for (int i = 0; i < size; ++i) {
		assert(!isnan(array[i]) && isfinite(array[i]));
	}
}

void Solver::printArray(string name, int size, VectorXf array) {
	if (DEBUG) {
		cout << endl << name << ": ";
		for (int i = 0; i < size; i++) {
			cout << array[i] << ", ";
		}
	}
}

void Solver::printMatrix(string name, int width, int height, MatrixXf matrix) {
	if (DEBUG) {
		cout << endl << name << ": ";
		for (int i = 0; i < width; i++) {
			cout << endl;
			for (int j = 0; j < height; j++) {
				cout << matrix(i, j) << ", ";
			}
		}
	}
}