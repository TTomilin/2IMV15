#include "RodConstraint.h"
#include "GL/glut.h"

RodConstraint::RodConstraint(Particle* p1, Particle* p2, const double dist, const int id) :
    Constraint(0, 0), m_p1(p1), m_p2(p2), m_dist_squared(dist * dist) {

    const int width = 1;
    const int height = N_DIMENSIONS;
    particles.push_back(p1);
    particles.push_back(p2);
}
void RodConstraint::draw() const
{
  glBegin( GL_LINES );
  glColor3f(0.8, 0.7, 0.6);
  glVertex3f( m_p1->m_Position[0], m_p1->m_Position[1], m_p1->m_Position[2] );
  glColor3f(0.8, 0.7, 0.6);
  glVertex3f( m_p2->m_Position[0], m_p2->m_Position[1], m_p2->m_Position[2] );
  glEnd();
}

/**
 * Compute the C of this constraint
 * @return x * x - l * l
 */
double RodConstraint::getC() {
    Vec3 distance = m_p1->m_Position - m_p2->m_Position;
    return norm2(distance) - m_dist_squared;
}

/**
 * Computes the CDot of this constraint
 * @return 2x * 2xd
 */
double RodConstraint::getCDot() {
    Vec3 distance_shift = (m_p1->m_Position - m_p2->m_Position) * 2.0;
    Vec3 velocity_shift = (m_p1->m_Velocity - m_p2->m_Velocity) * 2.0;
    return distance_shift * velocity_shift;
}

/**
 * Computes the J of this constraint
 * @return
 */
vector<Vector3f> RodConstraint::getJ() {
    vector<Vector3f> J;
    Vec3 position_shift_1 = (m_p1->m_Position - m_p2->m_Position) * 2.0;
    Vec3 position_shift_2 = (m_p2->m_Position - m_p1->m_Position) * 2.0;
    J.push_back(Vector3f(position_shift_1[0], position_shift_1[1], position_shift_1[2]));
    J.push_back(Vector3f(position_shift_2[0], position_shift_2[1], position_shift_2[2]));
    return J;
}

/**
 * Computes the JDot of this constraint
 * @return
 */
vector<Vector3f> RodConstraint::getJDot() {
    vector<Vector3f> JDot;
    Vec3 velocity_shift_1 = (m_p1->m_Velocity - m_p2->m_Velocity) * 2.0;
    Vec3 velocity_shift_2 = (m_p2->m_Velocity - m_p1->m_Velocity) * 2.0;
    JDot.push_back(Vector3f(velocity_shift_1[0], velocity_shift_1[1], velocity_shift_1[2]));
    JDot.push_back(Vector3f(velocity_shift_2[0], velocity_shift_2[1], velocity_shift_2[2]));
    return JDot;
}

void RodConstraint::addToJx(MatX Jx) {
    // Constraints and implicit not supported together
}

void RodConstraint::addToJv(MatX Jv) {
    // Constraints and implicit not supported together
}
