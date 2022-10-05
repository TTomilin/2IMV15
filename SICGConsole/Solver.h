#pragma once
#include "Particle.h"
#include "Constraint.h"
#include "JacobianMatrix.h"
#include <vector>
#include <map>
#include <unordered_map>
#include <set>
#include "Mouse.h"
#include "RigidBody.h"
#include "Contact.h"
#include "System.h"
#include "MatX.h"

using namespace std;

class Solver {

public:
    struct Vec3Tuple {
        Vec3 vec1;
        Vec3 vec2;
    };

    Solver(const vector<Particle*>& particles, const vector<RigidBody*>& bodies, 
        const vector<Force*>& forces, const vector<Force*>& constraints,
        const double ks, const double kd, const double epsilon, Mouse& mouse);

    const vector<Particle*>& m_Particles;
    const vector<RigidBody*>& m_Bodies;
    const vector<Force*>& m_Forces;
    const vector<Force*>& m_Constraints;
  
    const double m_ks;
    const double m_kd;
    const double m_epsilon;
    Mouse& m_mouse;

    void simulation_step(double dt);
    void particleDerivative(vector<Vec3Tuple>& forceTuples);
    void bodyDerivative(vector<RigidBody::State>& bodyState);
    void applyDerivative(vector<Vec3Tuple> derivatives, vector<RigidBody::State> body_derivatives, double dt);
    void clearForces(const vector<Particle*>& particles);
    void calculateForces(const vector<Particle*>& particles, const vector<Force*>& forces);
    void calculateForce(const vector<Particle*>& particles, Force* force);
    void constrainForces();
    void applyNewVelocities(vector<Vec3Tuple>& forceTuples);
    void assertArray(double* array, int size);
    void printArray(string name, int size, VectorXf array);
    void printMatrix(string name, int width, int height, MatrixXf matrix);
	MatX calculateJacobianX(const vector<Particle*>& particles, const vector<Force*>& forces);
	MatX calculateJacobianV(const vector<Particle*>& particles, const vector<Force*>& forces);
    Vec3 pt_velocity(IObject* obj, Vec3 p);
    bool collision(Contact* c, double epsilon);

	//void find_all_collisions(vector<Contact>& contacts);
};