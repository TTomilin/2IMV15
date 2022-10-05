#include "Particle.h"
#include "GL/glut.h"

bool DEBUG = false;

Particle::Particle(const Vec3& ConstructPos, const Vec3 massInv):
	m_ConstructPos(ConstructPos),
	m_Position(ConstructPos),
	m_Velocity(Vec3(0.0, 0.0, 0.0)),
	m_AccumulatedForce(Vec3(0.0, 0.0, 0.0)),
	m_MassInv(massInv),
	m_ID(0)
{
	color = Vec3(1.0, 1.0, 1.0);
	movable = true;
}

Particle::Particle(const Vec3& ConstructPos, int m_ID, const Vec3 massInv) :
	Particle(ConstructPos, massInv)
{
	this->m_ID = m_ID;
}

Particle::Particle(const Vec3& ConstructPos, int m_ID, const Vec3 massInv, const bool _movable):
	Particle(ConstructPos, m_ID, massInv)
{
	movable = _movable;
}

Particle::~Particle(void)
{
}

void Particle::reset()
{
	m_Position = m_ConstructPos;
	m_Velocity = Vec3(0.0, 0.0, 0.0);
	color = Vec3(1.0, 1.0, 1.0);
}

void Particle::draw(bool drawForcesOnParticles)
{
	glPushMatrix();

	glColor3f(color[0], color[1], color[2]);
	GLUquadric* quad = gluNewQuadric();
	glTranslatef(m_Position[0], m_Position[1], m_Position[2]);
	gluSphere(quad, radius, 50, 50);

	glPopMatrix();

	if (drawForcesOnParticles) {
		glBegin(GL_LINES);
		glColor3f(.2, .2, 1.); // Force color
		glVertex3f(m_Position[0], m_Position[1], m_Position[2]);
		Vec3 arrow = m_Position + 10. * m_AccumulatedForce;
		glVertex3f(arrow[0], arrow[1], arrow[2]);
		glEnd();
	}
}

void Particle::print_data(Particle particle)
{
	printf("\n\nParticle (ID): %3d", particle.m_ID);
	printf("\n\tConstruction position: (% 6.5f, % 6.5f, % 6.5f)", particle.m_ConstructPos[0], particle.m_ConstructPos[1], particle.m_ConstructPos[2]);
	printf("\n\tCurrent position     : (% 6.5f, % 6.5f, % 6.5f)", particle.m_Position[0], particle.m_Position[1], particle.m_Position[2]);
	printf("\n\tCurrent velocity     : (% 6.5f, % 6.5f, % 6.5f)", particle.m_Velocity[0], particle.m_Velocity[1], particle.m_Velocity[2]);
	printf("\n\tCurrent force (sum)  : (% 6.5f, % 6.5f, % 6.5f)", particle.m_AccumulatedForce[0], particle.m_AccumulatedForce[1], particle.m_AccumulatedForce[2]);
	printf("\n\tInverted Mass        : (% 6.5f, % 6.5f, % 6.5f)", particle.m_MassInv[0], particle.m_MassInv[1], particle.m_MassInv[2]);
}

vector<double> Particle::getBoundingBox()
{
	vector<double> bounds;
	for (uint8_t axis = 0; axis < 3; axis++) {
		bounds.push_back(m_Position[axis] - radius);
		bounds.push_back(m_Position[axis] + radius);
	}
	return bounds;
}
