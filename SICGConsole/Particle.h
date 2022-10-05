#pragma once

#include <iostream>
#include "include\gfx\vec2.h"
#include "include/gfx/vec3.h"
#include "IObject.h"


extern bool DEBUG;

class Particle : public IObject
{
public:
	const double radius = 0.02;
	Vec3 m_MassInv;

	Particle(const Vec3& ConstructPos, const Vec3 massInv);
	Particle(const Vec3& ConstructPos, int m_ID, const Vec3 massInv);
	Particle(const Vec3& ConstructPos, int m_ID, const Vec3 massInv, const bool _movable);
	virtual ~Particle(void);

	void reset();
	void draw(bool drawForcesOnParticles);

	int m_ID;
	Vec3 m_ConstructPos;
	Vec3 m_Position;
	Vec3 m_Velocity;
	Vec3 m_AccumulatedForce;
	Vec3 color;
	bool movable;

	static void print_data(Particle particle);

	vector<double> getBoundingBox() override;

	int getID() override { return m_ID; }
	Vec3 getPosition() override { return m_Position; }
	Mat3 getIinv() override {
		return Mat3(
			Vec3(pow(radius, 2), 0, 0),
			Vec3(0, pow(radius, 2), 0),
			Vec3(0, 0, pow(radius, 2))
		);
	}

	double getMassSum() override {return m_MassInv[0] + m_MassInv[1] + m_MassInv[2]; };  // Temporary fix for collision detection
	Vec3 getMass() override { return m_MassInv; }
	Vec3 getLinearMomentum() override { return m_AccumulatedForce; }		// Not sure about this
	void addLinearMomentum(Vec3 amount) { m_AccumulatedForce += amount; };	// Not sure about this
	void setVelocity(Vec3 amount) override { m_Velocity = amount; }
	Vec3 getVelocity() override { return m_Velocity; }
	Vec3 getAngularMomentum() override { return Vec3(0, 0, 0); }
	void addAngularMomentum(Vec3 amount) { /* NONE */ };
	void setOmega(Vec3 amount) override { /* NONE */ }

	Type getType() override { return Type::particle; }
private:

};
