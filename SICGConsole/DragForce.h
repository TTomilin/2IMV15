#pragma once
#include "Force.h"

class DragForce : public Force {
public:
    DragForce();
    DragForce(double k_drag);
    void draw() const override;
    void apply(const std::vector<Particle*>& particles) override;
    void addToJx(MatX Jx) override;
    void addToJv(MatX Jv) override;
private:
    double K_DRAG = -0.015;
};
