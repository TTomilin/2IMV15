#pragma once
#include "Force.h"

class AngularSpring : public Force {

public:
    AngularSpring(Particle* mass_point, Particle* p1, Particle* p2, double angle, double ks, double kd);
    void draw() const override;
    void apply(const std::vector<Particle*>& particles) override;

    void addToJx(MatX Jx) override;
    void addToJv(MatX Jv) override;

private:
    Particle* const m_mass_point; // The mass point
    Particle* const m_p1;         // Particle 1
    Particle* const m_p2;         // Particle 2
    double const m_angle;         // Cosine of rest angle
    double const m_ks, m_kd;      // Spring strength constants

    void calculate_force(Particle* p, double angle_delta);
    void apply_force(Particle* p, Vec3 force);
    double clip(double value, double lower, double upper);
    Vec3 rotate_around_particle(const Vec3 pivot, const Vec3 pos, const double angle);
};

