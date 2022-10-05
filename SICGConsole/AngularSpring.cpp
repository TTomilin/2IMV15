#include "stdafx.h"
#include "AngularSpring.h"
#include <algorithm>


AngularSpring::AngularSpring(Particle* massPoint, Particle* p1, Particle* p2, double angle, double ks, double kd) :
    m_mass_point(massPoint), m_p1(p1), m_p2(p2), m_angle(angle), m_ks(ks), m_kd(kd) {
}

void AngularSpring::draw() const {
    glBegin(GL_LINES);
    glColor3f(0.3, 0.3, 1);
    glVertex3f(m_mass_point->m_Position[0], m_mass_point->m_Position[1], m_mass_point->m_Position[2]);
    glVertex3f(m_p1->m_Position[0], m_p1->m_Position[1], m_p1->m_Position[2]);
    glColor3f(0.3, 0.3, 1);
    glVertex3f(m_mass_point->m_Position[0], m_mass_point->m_Position[1], m_mass_point->m_Position[2]);
    glVertex3f(m_p2->m_Position[0], m_p2->m_Position[1], m_p2->m_Position[2]);
    glEnd();
}

void AngularSpring::apply(const std::vector<Particle*>& particles) {
    Vec3 distance1 = m_p1->m_Position - m_mass_point->m_Position;
    Vec3 distance2 = m_p2->m_Position - m_mass_point->m_Position;

    double cos_angle = clip(distance1 * distance2 / (norm(distance1) * norm(distance2)), -1.0, 1.0); // Clip for rounding errors
    double current_angle = acos(cos_angle);
    double angle_delta = (m_angle - current_angle) / 2;

    calculate_force(m_p1, -angle_delta);
    calculate_force(m_p2, angle_delta);
}

double AngularSpring::clip(double value, double lower, double upper) {
    return std::max(lower, std::min(value, upper));
}

void AngularSpring::calculate_force(Particle* p, double angle_delta) {
    Vec3 rotation = rotate_around_particle(m_mass_point->m_Position, p->m_Position, angle_delta);
    Vec3 distance_shift = rotation - p->m_Position;

    double distance_shift_norm = norm(distance_shift);

    Vec3 velocity_diff = p->m_Velocity - m_mass_point->m_Velocity;
    Vec3 force = -((m_ks * distance_shift_norm) + m_kd * (velocity_diff * distance_shift / distance_shift_norm)) * distance_shift_norm;

    apply_force(p, force);
}

Vec3 AngularSpring::rotate_around_particle(const Vec3 pivot, const Vec3 position, const double angle) {
    double sin_val = sin(angle);
    double cos_val = cos(angle);
    Vec3 delta = position - pivot;

    double x_new = delta[0] * cos_val - delta[1] * sin_val + delta[2] * sin_val;
    double y_new = delta[0] * sin_val + delta[1] * cos_val - delta[2] * sin_val;
    double z_new = delta[0] * sin_val - delta[1] * sin_val + delta[2] * cos_val;

    return Vec3(x_new + pivot[0], y_new + pivot[1], z_new + pivot[2]);
}

void AngularSpring::apply_force(Particle* p, Vec3 force) {
    printVector(force, "New Angular Spring Force");
    assertVector(force);
    p->m_AccumulatedForce += force;
    m_mass_point->m_AccumulatedForce -= force;
}

void AngularSpring::addToJx(MatX Jx) {
    // TODO
}

void AngularSpring::addToJv(MatX Jv) {
    // TODO
}