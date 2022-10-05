#include "stdafx.h"
#include "GravityForce.h"
#include <assert.h>

void GravityForce::draw() const{}

void GravityForce::apply(const std::vector<Particle*>& particles) {
    for (int i = 0; i < particles.size(); ++i) {
        if (particles[i]->m_MassInv[1] > 0.) {
            particles[i]->m_AccumulatedForce[1] += m_Gravity[1] / particles[i]->m_MassInv[1];
        }
    }
}

void GravityForce::addToJx(MatX Jx) {
    // Force derived over position gives nothing
}

void GravityForce::addToJv(MatX Jv) {
    // Force derived over velocity gives nothing
}
