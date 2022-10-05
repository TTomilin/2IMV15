#pragma once
#include "Particle.h"
#include "RigidBody.h"
#include "SpringForce.h"
#include <vector>
#include "Camera.h"
#include "GL/glut.h"
#include "include/gfx/mat4.h"

using namespace std;

class Mouse
{
public:
	Mouse(vector<Particle*>& _particles, vector<RigidBody*>& _bodies, Camera& _cam);

	void updateButton(int button, int state, int x, int y);
	void updateCoords(int x, int y);
	Particle* getMouseParticle();
	SpringForce* getMouseSpring();

private:
	const double MOUSE_KS = 0.1;
	const double MOUSE_KD = 0.05;

	vector<Particle*>& particles;
	vector<RigidBody*>& bodies;
	Camera& cam;

	void selectObject(int x, int y);
	void deselectObject();

	void updateMouseParticle(int x, int y);
	void updateCamera(int x, int y);

	// Mouse Selection Helper Functions
	Vec3 closestPoint(Vec3 A, Vec3 B, Vec3 P, double* t);
	void raycast(int x, int y, Vec3& near, Vec3& far);

	// Selection related
	Particle* selectedParticle = nullptr;
	RigidBody* selectedBody = nullptr;
	SpringForce* mouseSpring = nullptr;
	Particle* mouseParticle = nullptr;
	double mouseParticle_t;
	Vec3 clickWorldPos;
	Vec3 bodyOriginalPos;
	Vec3 originalColor;
	bool originalMovable;

	// Camera movement related
	bool moveCamera = false;
	Vec2 screenClickPoint;
	Vec2 initialTiltAndPan;
	const double SENSITIVITY = 200.0;
};