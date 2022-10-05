#pragma once

#include "include/gfx/vec3.h"
#include <vector>
#include "include/gfx/mat3.h"
#include "include/gfx/quat.h"
#include "Particle.h"
#include "IObject.h"

using namespace std;

class RigidBody : public IObject
{
public:
	struct State {
		State() {};
		State(Vec3 _x, Quat _q, Vec3 _P, Vec3 _L) {
			x = Vec3(_x);
			q = Quat(_q);
			P = Vec3(_P);
			L = Vec3(_L);
		};
		Vec3 x;
		Quat q;
		Vec3 P, L;
	};

	RigidBody(Vec3& ConstructPos, Vec3 Mass, Vec3& Size, Vec3& Rotation, Vec3& Color, int m_ID);
	RigidBody(Vec3& ConstructPos, Vec3 Mass, Vec3& Size, Vec3& Rotation, Vec3& Color, int m_ID, bool _movable);
	int id;
	virtual ~RigidBody(void);

	void reset();

	// Setters and getters
	void setState(State state);
	State getDerivativeState();
	void moveBody(Vec3 pos);
	vector<Particle*> get_particles();
	vector<pair<Vec3, Vec3>> get_edges();
	vector<uint8_t> get_edges_of_particle(uint8_t id);

	// Base class methods
	void draw();

	// Particles of the body
	vector<Particle*> particles;	// First 8 particles are edge particles
	// Variables
	Vec3 x_original;
	Quat q_original;
	Vec3 size;
	Vec3 color;
	bool movable;
	// Constants
	Vec3 mass;
	Mat3 Ibody,
		Ibodyinv;
	// State variables
	Vec3 x;
	Quat q;
	Vec3 P,
		L;
	// Derived Quantities
	Mat3 Iinv,
		R;
	Vec3 v, omega;
	// Computed Quantities
	Vec3 force,
		torque;

	vector<double> getBoundingBox() override;

	int getID() override { return id; }
	Vec3 getPosition() override { return x; }
	Mat3 getIinv() override { return Iinv; }
	Vec3 getMass() override { return mass; }
	double getMassSum() override { return (mass[0] + mass[1] + mass[2]) / 3; };  // Temporary fix for collision detection
	Vec3 getLinearMomentum() override { return P; }
	void addLinearMomentum(Vec3 amount) override { P += amount; }
	void setVelocity(Vec3 amount) override { v = amount; }
	Vec3 getVelocity() override { return v; }
	Vec3 getAngularMomentum() override { return L; }
	void addAngularMomentum(Vec3 amount) override { L += amount; }
	void setOmega(Vec3 amount) override { omega = amount; }

	Type getType() override { return Type::rigidBody; }
private:
	// Internal computes
	void computeForce();
	void computeTorque();
	void computeAuxiliary();
	// Keep particles in correct position
	void setupParticles();
};