#pragma once
#include "linearSolver.h"
#include "JacobianMatrix.h"
#include "Particle.h"

class JWJTransposed : public implicitMatrixWithTrans {
private:
    const int n_particle_values;
    const double* W;
    JacobianMatrix& J;
    double* m_ValueVector;
public:
    JWJTransposed(const int n_particle_values, const double W[], JacobianMatrix& J);
    ~JWJTransposed();
    void matVecMult(double x[], double r[]) override;
    void matTransVecMult(double x[], double r[]) override;
};

