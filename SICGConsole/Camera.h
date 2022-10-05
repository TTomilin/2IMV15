#pragma once
#include "stdafx.h"
#include "Camera.h"
#include "include/gfx/vec3.h"
#include "math.h"

using namespace std;

class Camera
{
public:
	Camera();

	void buttonPressed(unsigned char key, int x, int y);
	void reset();

	// Setters
	void setTiltAndPan(Vec2 data);

	// Getters
	Vec3 getPosition();
	Vec3 getDirection();
	Vec3 getUp();
	Vec3 getRight();
	Vec2 getTiltAndPan();
	double getFOV();
	double getNearPlane();
	double getFarPlane();

private:
	Vec3 position;
	double tilt;
	double pan;
	double fov;

	const double MOVE_STEPSIZE = 0.05;
	const double TILT_STEPSIZE = M_PI / 6.0;
	const double PAN_STEPSIZE = M_PI / 6.0;
	const double FOV_STEPSIZE = 5;
	const double START_TILT = M_PI / 2.0;
	const double START_PAN = -M_PI / 2.0;
	const double START_FOV = 90.0;
	
	const Vec3 GLOBAL_UP = Vec3(0.0, 1.0, 0.0);
	const Vec3 START_POS = Vec3(0.0, 0.0, 1.0);
	
	const double NEAR_PLANE = 0.01;
	const double FAR_PLANE = 100.0;
};

