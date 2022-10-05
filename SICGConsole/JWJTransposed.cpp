#include "stdafx.h"
#include "JWJTransposed.h"
#include <assert.h>

JWJTransposed::JWJTransposed(const int n_particle_values, const double W[], JacobianMatrix& J) : n_particle_values(n_particle_values), W(W), J(J) {
    m_ValueVector = new double[n_particle_values];
    fill(m_ValueVector, m_ValueVector + n_particle_values, 0.0);
}

void JWJTransposed::matVecMult(double x[], double r[]) {
    if (DEBUG) printf("\n\nJWJTransposed Multiplication");
    J.matTransVecMult(x, m_ValueVector);
    for (int i = 0; i < n_particle_values; ++i) {
        m_ValueVector[i] *= W[i];
        assert(!isnan(m_ValueVector[i]) && isfinite(m_ValueVector[i]));
    }
    J.matVecMult(m_ValueVector, r);
}

void JWJTransposed::matTransVecMult(double x[], double r[]) {
    matVecMult(x, r); // JWJTranposed is symmetric so we can just call the default vector multiplication.
}

JWJTransposed::~JWJTransposed() {
    delete[] m_ValueVector;
    m_ValueVector = nullptr;
}