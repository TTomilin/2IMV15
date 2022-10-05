#include "stdafx.h"
#include "DragForce.h"

DragForce::DragForce() {}

DragForce::DragForce(double k_drag): K_DRAG(k_drag) {}

void DragForce::draw() const {}

void DragForce::apply(const std::vector<Particle*>& particles) {
    for (int i = 0; i < particles.size(); ++i) {
        particles[i]->m_AccumulatedForce += K_DRAG * particles[i]->m_Velocity;
    }
}

void DragForce::addToJx(MatX Jx) {
    // None
}

void DragForce::addToJv(MatX Jv) {
    for (int i = 0; i < Jv.m_size / 3; i++) {
        Jv.addToSection(i, i, K_DRAG * Mat3::I());
    }
}
