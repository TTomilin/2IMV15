#pragma once
#include "Force.h"

class GravityForce : public Force {
public:
    void draw() const override;
    void apply(const std::vector<Particle*>& particles) override;
    void addToJx(MatX Jx) override;
    void addToJv(MatX Jv) override;
private:
    const Vec3 m_Gravity = Vec3(0.0, -9.80665 * pow(10, -3.5), 0.0);
};
