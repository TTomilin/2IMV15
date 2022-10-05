#include "SpringForce.h"
#include "GL/glut.h"
#include <assert.h>


SpringForce::SpringForce(Particle *p1, Particle * p2, double dist, double ks, double kd) :
  m_p1(p1), m_p2(p2), m_dist(dist), m_ks(ks), m_kd(kd) {}

void SpringForce::apply(const std::vector<Particle*>& particles) {
    Vec3 distance = m_p1->m_Position - m_p2->m_Position;
    Vec3 velocity_diff = m_p1->m_Velocity - m_p2->m_Velocity;
    printVector(distance, "Spring Force Position Differences");
    printVector(velocity_diff, "Spring Force Velocity Differences");

    Vec3 dist_norm = distance / norm(distance);
    double magnitude = norm(distance);
    double scale = magnitude == 0 ? m_ks * m_dist : -(m_ks * (magnitude - m_dist) + m_kd * (velocity_diff * dist_norm / magnitude));

    Vec3 force1 = scale * distance;
    Vec3 force2 = -force1;

    m_p1->m_AccumulatedForce += force1;
    m_p2->m_AccumulatedForce += force2;
    printVector(force1, "New Spring Force 1");
    printVector(force2, "New Spring Force 2");
}

void SpringForce::addToJx(MatX Jx) {
    Vec3 dist = m_p1->m_Position - m_p2->m_Position;
    Mat3 dxtdx = Mat3::outer_product(dist);
    double length = norm(dist);
    if (length != 0) length = 1. / length;

    dxtdx *= length * length;
    Mat3 mat = (dxtdx + (Mat3::I() - dxtdx) * (1 - m_dist * length)) * -m_ks;

    Jx.addToSection(m_p1->m_ID, m_p2->m_ID, mat);


    dist = m_p2->m_Position - m_p1->m_Position;
    dxtdx = Mat3::outer_product(dist);
    length = norm(dist);
    if (length != 0) length = 1. / length;

    dxtdx *= length * length;
    mat = (dxtdx + (Mat3::I() - dxtdx) * (1 - m_dist * length)) * -m_ks;

    Jx.addToSection(m_p2->m_ID, m_p1->m_ID, mat);
}

void SpringForce::addToJv(MatX Jv) {
    // TODO
}

void SpringForce::draw() const
{
    glBegin(GL_LINES);
    glColor3f(0.6, 0.7, 0.8);
    glVertex3f(m_p1->m_Position[0], m_p1->m_Position[1], m_p1->m_Position[2]);
    glColor3f(0.6, 0.7, 0.8);
    glVertex3f(m_p2->m_Position[0], m_p2->m_Position[1], m_p2->m_Position[2]);
    glEnd();
}
