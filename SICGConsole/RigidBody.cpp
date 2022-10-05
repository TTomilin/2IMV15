#include "stdafx.h"
#include "RigidBody.h"
#include "GL/glut.h"

//#define DEBUG

RigidBody::RigidBody(Vec3& ConstructPos, Vec3 Mass, Vec3& Size, Vec3& Rotation, Vec3& Color, int m_ID) :
	x_original(ConstructPos), size(Size), color(Color)
{
	id = m_ID;
	q_original = Quat::fromEuler(Rotation);
	mass = Mass;
	movable = true;

	Ibody = (getMassSum() / 12.0) * Mat3(				// Ibody Matrix
		Vec3(pow(size[1], 2) + pow(size[2], 2), 0, 0),
		Vec3(0, pow(size[0], 2) + pow(size[2], 2), 0),
		Vec3(0, 0, pow(size[0], 2) + pow(size[1], 2))
	);
	Ibodyinv = Ibody.inverse();					// Ibody Matrix inverse

	// Create Particles of the body
	for (uint8_t i = 0; i < 27; i++) {
		// The body id will be m_ID and the particles id will follow the body id
		// (+1 to avoid body and particle id overlapping)
		particles.push_back(new Particle(Vec3(0.0, 0.0, 0.0), m_ID + particles.size() + 1, 1 / (getMassSum() / 27.0), false));
	}
	setupParticles();
}

RigidBody::RigidBody(Vec3& ConstructPos, Vec3 Mass, Vec3& Size, Vec3& Rotation, Vec3& Color, int m_ID, bool _movable) :
	RigidBody(ConstructPos, Mass, Size, Rotation, Color, m_ID)
{
	movable = _movable;
}

RigidBody::~RigidBody(void)
{

}

void RigidBody::reset()
{
	// Reset body
	x = x_original;
	q = q_original;
	P = Vec3(0.0, 0.0, 0.0);
	L = Vec3(0.0, 0.0, 0.0);
	force = Vec3(0.0, 0.0, 0.0);
	torque = Vec3(0.0, 0.0, 0.0);
	computeAuxiliary();
	// Reset all particles
	for (int i = 0; i < particles.size(); i++) particles.at(i)->reset();
	setupParticles();
}

void RigidBody::moveBody(Vec3 pos)
{
	x = pos;
	setupParticles();
}

void RigidBody::setState(State state)
{
	x = state.x;
	q = state.q;
	P = state.P;
	L = state.L;

	computeAuxiliary();
	for (int i = 0; i < particles.size(); i++) {
		particles.at(i)->m_Position = (R * particles.at(i)->m_ConstructPos) + x;
	}
}

RigidBody::State RigidBody::getDerivativeState()
{
	computeForce();
	computeTorque();
	return State(
		Vec3(v),
		0.5 * (Quat(omega[0], omega[1], omega[2], 0.0) * q),
		Vec3(force),
		Vec3(torque)
	);
}

vector<Particle*> RigidBody::get_particles()
{
	return particles;
}

vector<pair<Vec3, Vec3>> RigidBody::get_edges()
{
	vector<pair<Vec3, Vec3>> edges;
	// Front face
	edges.push_back(make_pair(particles[0]->getPosition(), particles[1]->getPosition()));	// AB
	edges.push_back(make_pair(particles[1]->getPosition(), particles[3]->getPosition()));	// BD
	edges.push_back(make_pair(particles[3]->getPosition(), particles[2]->getPosition()));	// DC
	edges.push_back(make_pair(particles[2]->getPosition(), particles[0]->getPosition()));	// CA
	// Back face
	edges.push_back(make_pair(particles[4]->getPosition(), particles[5]->getPosition()));	// EF
	edges.push_back(make_pair(particles[5]->getPosition(), particles[7]->getPosition()));	// FH
	edges.push_back(make_pair(particles[7]->getPosition(), particles[6]->getPosition()));	// HG
	edges.push_back(make_pair(particles[6]->getPosition(), particles[4]->getPosition()));	// GE
	// Side faces
	edges.push_back(make_pair(particles[7]->getPosition(), particles[3]->getPosition()));	// HD
	edges.push_back(make_pair(particles[6]->getPosition(), particles[2]->getPosition()));	// GC
	edges.push_back(make_pair(particles[5]->getPosition(), particles[1]->getPosition()));	// FB
	edges.push_back(make_pair(particles[4]->getPosition(), particles[0]->getPosition()));	// EA
	return edges;
}

vector<uint8_t> RigidBody::get_edges_of_particle(uint8_t id)
{
	vector<uint8_t> edges;
	switch (id) {
	case 0:
		edges.push_back(1);
		edges.push_back(2);
		edges.push_back(4);
		break;
	case 1:
		edges.push_back(0);
		edges.push_back(3);
		edges.push_back(5);
		break;
	case 2:
		edges.push_back(0);
		edges.push_back(3);
		edges.push_back(6);
		break;
	case 3:
		edges.push_back(1);
		edges.push_back(2);
		edges.push_back(7);
		break;
	case 4:
		edges.push_back(0);
		edges.push_back(5);
		edges.push_back(6);
		break;
	case 5:
		edges.push_back(1);
		edges.push_back(4);
		edges.push_back(7);
		break;
	case 6:
		edges.push_back(2);
		edges.push_back(4);
		edges.push_back(7);
		break;
	case 7:
		edges.push_back(3);
		edges.push_back(5);
		edges.push_back(6);
		break;
	}
	return edges;
}

void RigidBody::computeAuxiliary()
{
	// Normalize quaternion
	double div = 0.0;
	for (uint8_t i = 0; i < 3; i++) {
		div += q.vector()[i] * q.vector()[i];
	}
	div += q.scalar() * q.scalar();
	q /= div;
	// Compute
	R = quat_to_matrix(q);
	v = P / getMassSum();
	Iinv = R * Ibodyinv * transpose(R);
	omega = Iinv * L;
}

vector<double> RigidBody::getBoundingBox()
{
	vector<double> bounds{ DBL_MAX,-DBL_MAX,DBL_MAX,-DBL_MAX,DBL_MAX,-DBL_MAX };
	for (uint8_t i = 0; i < 8; i++) {
		for (uint8_t axis = 0; axis < 3; axis++) {
			if (particles[i]->m_Position[axis] < bounds[axis * 2]) {			//Min
				bounds[axis * 2] = particles[i]->m_Position[axis];
			}
			else if (bounds[axis * 2 + 1] < particles[i]->m_Position[axis]) {	//Max
				bounds[axis * 2 + 1] = particles[i]->m_Position[axis];
			}
		}
	}
	return bounds;
}

void RigidBody::computeForce()
{
	force = Vec3(0.0, 0.0, 0.0);
	for (Particle* p : particles) {
		force += p->m_AccumulatedForce;
	}
}

void RigidBody::computeTorque()
{
	torque = Vec3(0.0, 0.0, 0.0);
	for (Particle* p : particles) {
		torque += cross((p->m_Position - x), p->m_AccumulatedForce);
	}
}

void RigidBody::draw()
{
#ifdef DEBUG
	for (int i = 0; i < particles.size(); i++) {
		particles.at(i)->draw(false);
	}
#endif
	glPushMatrix();

	// Body color
	glColor3f(color[0], color[1], color[2]);
	// Move body
	glTranslatef(x[0], x[1], x[2]);
	// Rotate body
	GLdouble rot_mat[16] = {
		R[0][0], R[1][0], R[2][0], 0,
		R[0][1], R[1][1], R[2][1], 0,
		R[0][2], R[1][2], R[2][2], 0,
		0, 0, 0, 1,
	};
	glMultMatrixd(rot_mat);
	// Draw body (Faces order top, bottom, right, left, front, rear)
	Vec3 halfSize = this->size / 2.0;
	glBegin(GL_QUADS);
	// Top Face
	glVertex3f(halfSize[0], halfSize[1], -halfSize[2]);
	glVertex3f(-halfSize[0], halfSize[1], -halfSize[2]);
	glVertex3f(-halfSize[0], halfSize[1], halfSize[2]);
	glVertex3f(halfSize[0], halfSize[1], halfSize[2]);
	// Bottom Face
	glVertex3f(halfSize[0], -halfSize[1], halfSize[2]);
	glVertex3f(-halfSize[0], -halfSize[1], halfSize[2]);
	glVertex3f(-halfSize[0], -halfSize[1], -halfSize[2]);
	glVertex3f(halfSize[0], -halfSize[1], -halfSize[2]);
	// Right Face
	glVertex3f(halfSize[0], halfSize[1], -halfSize[2]);
	glVertex3f(halfSize[0], halfSize[1], halfSize[2]);
	glVertex3f(halfSize[0], -halfSize[1], halfSize[2]);
	glVertex3f(halfSize[0], -halfSize[1], -halfSize[2]);
	// Left Face
	glVertex3f(-halfSize[0], halfSize[1], halfSize[2]);
	glVertex3f(-halfSize[0], halfSize[1], -halfSize[2]);
	glVertex3f(-halfSize[0], -halfSize[1], -halfSize[2]);
	glVertex3f(-halfSize[0], -halfSize[1], halfSize[2]);
	// Front Face
	glVertex3f(halfSize[0], halfSize[1], halfSize[2]);
	glVertex3f(-halfSize[0], halfSize[1], halfSize[2]);
	glVertex3f(-halfSize[0], -halfSize[1], halfSize[2]);
	glVertex3f(halfSize[0], -halfSize[1], halfSize[2]);
	// Rear Face
	glVertex3f(halfSize[0], -halfSize[1], -halfSize[2]);
	glVertex3f(-halfSize[0], -halfSize[1], -halfSize[2]);
	glVertex3f(-halfSize[0], halfSize[1], -halfSize[2]);
	glVertex3f(halfSize[0], halfSize[1], -halfSize[2]);
	glEnd();

	glPopMatrix();
}

void RigidBody::setupParticles()
{
	uint8_t i = 8, edgeI = 0;
	Vec3 halfSize = size / 2.0;
	for (int8_t x = -1; x <= 1; x += 1) {
		for (int8_t y = -1; y <= 1; y += 1) {
			for (int8_t z = -1; z <= 1; z += 1) {
				// Ignore center particle
				if ((x != 0) || (y != 0) || (z != 0)) {
					Vec3 pos = Vec3(
						halfSize[0] * x,
						halfSize[1] * y,
						halfSize[2] * z
					);
					// These are edge particles
					// Edge particles must be at the 1st 8 particle indeces
					if ((x == -1 || x == 1) && (y == -1 || y == 1) && (z == -1 || z == 1)) {
						particles.at(edgeI)->m_ConstructPos = pos;
						edgeI++;
					}
					// These are in-edge particles
					else {
						particles.at(i)->m_ConstructPos = pos;
						i++;
					}
				}
				else {
					particles.at(particles.size() - 1)->m_ConstructPos = Vec3(0, 0, 0);
				}
			}
		}
	}

	for (i = 0; i < particles.size(); i++) {
		particles.at(i)->m_Position = (R * particles.at(i)->m_ConstructPos) + x;
	}
}
