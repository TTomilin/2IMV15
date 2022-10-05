#include "CircularWireConstraint.h"
#include "GL/glut.h"

#define PI 3.1415926535897932384626433832795

static void draw_circle(const Vec3 & vect, double radius)
{
	glBegin(GL_LINE_LOOP);
	glColor3f(0.0,1.0,0.0); 
	for (int i=0; i<360; i=i+18)
	{
		double degInRad = i*PI/180.;
		glVertex3f(vect[0]+cos(degInRad)*radius,vect[1]+sin(degInRad)*radius, vect[2]);
	}
	glEnd();
    glBegin(GL_LINE_LOOP);
    glColor3f(0.0, 1.0, 0.0);
    for (int i = 0; i < 360; i = i + 18)
    {
        double degInRad = i * PI / 180.;
        glVertex3f(vect[0] + cos(degInRad) * radius, vect[1], vect[2] + sin(degInRad) * radius);
    }
    glEnd();
    glBegin(GL_LINE_LOOP);
    glColor3f(0.0, 1.0, 0.0);
    for (int i = 0; i < 360; i = i + 18)
    {
        double degInRad = i * PI / 180.;
        glVertex3f(vect[0], vect[1] + cos(degInRad) * radius, vect[2] + sin(degInRad) * radius);
    }
    glEnd();
}

CircularWireConstraint::CircularWireConstraint(Particle *p, const Vec3 & center, const double radius, int index):
	Constraint(0, 0), m_p(p), m_center(center), m_radius(radius), m_radiusSquared(radius* radius) {

    const int width = 1;
    const int height = N_DIMENSIONS;
    particles.push_back(p);
}

void CircularWireConstraint::draw() const {
	draw_circle(m_center, m_radius);
}

/**
 * Compute the C of this constraint
 * @return x * x - r * r
 */
double CircularWireConstraint::getC() {
    Vec3 distance = m_p->m_Position - m_center;
    return norm2(distance) - m_radiusSquared;
}

/**
 * Computes the CDot of this constraint
 * @return 2 * x * xd
 */
double CircularWireConstraint::getCDot() {
    Vec3 distance = m_p->m_Position - m_center;
    Vec3 velocity = m_p->m_Velocity;
    return 2.0 * distance * velocity;
}

/**
 * Computes the J of this constraint
 * @return
 */
vector<Vector3f> CircularWireConstraint::getJ() {
    vector<Vector3f> J;
    Vec3 position_shift = 2.0 * (m_p->m_Position - m_center);
    J.push_back(Vector3f(position_shift[0], position_shift[1], position_shift[2]));
    return J;
}

/**
 * Computes the JDot of this constraint
 * @return
 */
vector<Vector3f> CircularWireConstraint::getJDot() {
    vector<Vector3f> JDot;
    Vec3 velocity_shift = 2.0 * m_p->m_Velocity;
    JDot.push_back(Vector3f(velocity_shift[0], velocity_shift[1], velocity_shift[2]));
    return JDot;
}

void CircularWireConstraint::addToJx(MatX Jx) {
    // Constraints and implicit not supported together
}

void CircularWireConstraint::addToJv(MatX Jv) {
    // Constraints and implicit not supported together
}
