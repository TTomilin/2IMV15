#pragma once
#include "Force.h"
#include "assert.h"
#include <Eigen/Dense>

#define N_DIMENSIONS 3

using namespace Eigen;

class Constraint : public Force {
protected:
	double m_C;
	double m_CDot;
	vector<Particle*> particles;

	void assertVector(Vec3 vector) const {
		for (int i = 0; i < N_DIMENSIONS; ++i) {
			assert(!isnan(vector[i]) && isfinite(vector[1]));
		}
	}
public:
	Constraint(const double C, const double CDot) : m_C(C), m_CDot(CDot) {}
	virtual ~Constraint() {};

	virtual double getC() = 0;
	virtual double getCDot() = 0;
	virtual vector<Vector3f> getJ() = 0;
	virtual vector<Vector3f> getJDot() = 0;

	vector<Particle*> getParticles() { return particles; };

	void apply(const vector<Particle*>& particles) {

	}
};

