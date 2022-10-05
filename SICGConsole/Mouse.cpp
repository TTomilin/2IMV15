#include "stdafx.h"
#include "Mouse.h"
#include "RigidBody.h"

Mouse::Mouse(vector<Particle*>& _particles, vector<RigidBody*>& _bodies, Camera& _cam) :
	particles{ _particles }, bodies{ _bodies }, cam{ _cam }
{

}

void Mouse::updateButton(int button, int state, int x, int y)
{
	if (button == GLUT_LEFT_BUTTON) {
		if (state == GLUT_DOWN) {
			selectObject(x, y);
		}
		else if (state == GLUT_UP) {
			deselectObject();
		}
	}
	if (button == GLUT_RIGHT_BUTTON) {
		if (state == GLUT_DOWN) {
			screenClickPoint = Vec2(y, x);
			initialTiltAndPan = Vec2(cam.getTiltAndPan()[0], cam.getTiltAndPan()[1]);
			moveCamera = true;
		}
		else if (state == GLUT_UP) {
			moveCamera = false;
		}
	}
}

void Mouse::updateCoords(int x, int y)
{
	updateMouseParticle(x, y);
	updateCamera(x, y);
}

Particle* Mouse::getMouseParticle()
{
	return mouseParticle;
}

SpringForce* Mouse::getMouseSpring()
{
	return mouseSpring;
}

void Mouse::selectObject(int x, int y) {
	Vec3 nearPos, farPos;
	raycast(x, y, nearPos, farPos);

	// Loop over particles
	double closestParticleT = DBL_MAX;
	Particle* closestParticle = nullptr;
	double closestParticleDist = DBL_MAX;
	for (Particle* obj : particles) {
		// How far from the camera is the intersection point of ray with sphere
		double *t = new double(DBL_MAX);
		// Calculate shortest distance between mouse ray and center of particle
		Vec3 res = closestPoint(nearPos, farPos, obj->m_Position, t);
		double dist = sqrt(
			pow(res[0] - obj->m_Position[0], 2) +
			pow(res[1] - obj->m_Position[1], 2) +
			pow(res[2] - obj->m_Position[2], 2)
		);
		// Check if this match is closer to the camera as well as intersecting the mouse ray
		if (*t >= 0.0 && *t <= 1.0) {
			double virtualRadius = obj->radius * (1 + obj->m_Velocity.magnitude() * 1000.0);
			if (dist <= virtualRadius && dist < closestParticleDist) {
				closestParticleDist = dist > obj->radius ? dist - obj->radius : 0.;
				closestParticleT = *t;
				closestParticle = obj;
			}
		}
	}
	// Loop over bodies
	double closestBodyT = DBL_MAX;
	double closestBodyDist = DBL_MAX;
	double closestBodyTmax = DBL_MAX;
	RigidBody* closestBody = nullptr;
	Vec3 closestBodyClickPos;
	for (RigidBody* obj : bodies) {
		// Get Body rotation matrix
		Mat3 rot = obj->R.inverse();
		// Rotate ray points in order to enforce AABB
		Vec3 origin = rot * (nearPos - obj->x);
		Vec3 farPos_r = rot * (farPos - obj->x);
		// Get Ray direction
		Vec3 dir = farPos_r - origin;
		Vec3 m_HalfSize = obj->size / 2.0;
		float lb[3] = { -m_HalfSize[0], -m_HalfSize[1], -m_HalfSize[2] };
		float rt[3] = { m_HalfSize[0],  m_HalfSize[1],  m_HalfSize[2] };
		// X
		float tx_lb = (lb[0] - origin[0]) / dir[0];
		float tx_rt = (rt[0] - origin[0]) / dir[0];
		// Y
		float ty_lb = (lb[1] - origin[1]) / dir[1];
		float ty_rt = (rt[1] - origin[1]) / dir[1];
		// Z
		float tz_lb = (lb[2] - origin[2]) / dir[2];
		float tz_rt = (rt[2] - origin[2]) / dir[2];

		float tMin = fmax(fmax(fmin(tx_lb, tx_rt), fmin(ty_lb, ty_rt)), fmin(tz_lb, tz_rt));
		float tMax = fmin(fmin(fmax(tx_lb, tx_rt), fmax(ty_lb, ty_rt)), fmax(tz_lb, tz_rt));

		if (!(tMax < 0 || tMin > tMax) && tMin < closestBodyT) {
			closestBodyT = tMin;
			closestBodyTmax = tMax;
			closestBodyDist = 0;
			closestBody = obj;
			closestBodyClickPos = nearPos + ((farPos - nearPos) * (closestBodyT + (closestBodyTmax - closestBodyT) / 2.0));
		}
	}

	// Select Object
	if (closestBodyDist < DBL_MAX || closestParticleT < DBL_MAX) {
		if (closestBodyDist < closestParticleDist || closestBodyT < closestParticleT) {
			selectedBody = closestBody;
			// Get body center particles
			selectedParticle = closestBody->get_particles().at(closestBody->get_particles().size() - 1);
			mouseParticle_t = closestBodyT + (closestBodyTmax - closestBodyT) / 2.0;
			// Highlight object
			originalColor = Vec3(closestBody->color);
			selectedBody->color = Vec3(1.0, 0.0, 0.0);
		}
		else if (closestBodyT > closestParticleT) {
			selectedParticle = closestParticle;
			mouseParticle_t = closestParticleT;
			// Highlight object
			originalColor = Vec3(closestParticle->color);
			selectedParticle->color = Vec3(1.0, 0.0, 0.0);
		}

		// Create Particle at mouse
		mouseParticle = new Particle(closestPoint(nearPos, farPos, selectedParticle->m_Position, new double(0)), 0);
		mouseParticle->m_Position = mouseParticle->m_ConstructPos;
		// Create spring between particle and mouse
		mouseSpring = new SpringForce(selectedParticle, mouseParticle, 0, MOUSE_KS, MOUSE_KD);
	}
}

void Mouse::deselectObject()
{
	if (selectedBody != nullptr) {
		// Reset Particle color
		selectedBody->color = Vec3(originalColor);
		selectedBody = nullptr;
	}

	if (selectedParticle != nullptr) {
		// Delete virtual particle on mouse cursor
		delete mouseParticle;
		mouseParticle = nullptr;
		// Reset Particle color
		selectedParticle->color = Vec3(originalColor);
		selectedParticle = nullptr;
		// Delete spring between particle and mouse
		delete mouseSpring;
		mouseSpring = nullptr;
	}
}

void Mouse::updateMouseParticle(int x, int y)
{
	if (selectedParticle != nullptr || selectedBody != nullptr) {
		// Raycast
		Vec3 nearPos, farPos;
		raycast(x, y, nearPos, farPos);
		// Update constraint
		mouseParticle->m_Position = nearPos + (farPos - nearPos) * mouseParticle_t;
	}
}

void Mouse::updateCamera(int x, int y)
{
	if (moveCamera) {
		Vec2 delta = Vec2((y - screenClickPoint[0]) / SENSITIVITY, (x - screenClickPoint[1]) / SENSITIVITY);
		cam.setTiltAndPan(Vec2(initialTiltAndPan[0] + delta[0], initialTiltAndPan[1] + delta[1]));
	}
}

////////////////////////////////////////////
// Helper functions for Mouse interaction //
////////////////////////////////////////////
void Mouse::raycast(int x, int y, Vec3& near, Vec3& far)
{
	GLint viewport[4];			// Viewport
	GLdouble modelview[16];		// Model View matrix
	GLdouble projection[16];	// Projection matrix
	GLfloat screenX, screenY;	// Mouse click screen coordinates
	// Retrieve Metrics
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
	glGetDoublev(GL_PROJECTION_MATRIX, projection);
	glGetIntegerv(GL_VIEWPORT, viewport);
	screenX = x; screenY = viewport[3] - y;
	// Retrieve raycast point on near and far plane
	gluUnProject(screenX, screenY, 0.0, modelview, projection, viewport, &near[0], &near[1], &near[2]);
	gluUnProject(screenX, screenY, 1.0, modelview, projection, viewport, &far[0], &far[1], &far[2]);
}

Vec3 Mouse::closestPoint(Vec3 A, Vec3 B, Vec3 P, double* t) {
	Vec3 AB = B - A;
	double ab_squared = dotProduct(AB, AB);
	Vec3 AP = P - A;
	double ap_dot_ab = dotProduct(AP, AB);
	// Retrieve closest point
	*t = (ap_dot_ab / ab_squared);
	Vec3 result = A + AB * (double)(*t);

	return result;
}
