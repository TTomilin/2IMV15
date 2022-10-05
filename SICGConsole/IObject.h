#pragma once
#include "include/gfx/mat3.h"
#include <vector>

using namespace std;

class IObject {
public:
	enum Type
	{
		particle,
		rigidBody
	};

	virtual vector<double> getBoundingBox() = 0; //xmin,xmax,ymin,ymax,zmin,zmax

	virtual int getID() = 0;
	virtual Vec3 getPosition() = 0;
	virtual Vec3 getMass() = 0;
	virtual double getMassSum() = 0;
	virtual Mat3 getIinv() = 0;
	virtual Vec3 getLinearMomentum() = 0;
	virtual void addLinearMomentum(Vec3 amount) = 0;
	virtual void setVelocity(Vec3 amount) = 0;
	virtual Vec3 getVelocity() = 0;
	virtual Vec3 getAngularMomentum() = 0;
	virtual void addAngularMomentum(Vec3 amount) = 0;
	virtual void setOmega(Vec3 amount) = 0;
	virtual Type getType() = 0;
};