#pragma once
#include "linearSolver.h"
#include "Particle.h"
#include <vector>

using namespace std;

struct MatrixBlock {
    const int particle_index;  
    const int constraint_index;
    const int width;
    const int height;
    double* const* data; // Array of value pointers

    MatrixBlock(const int constraint_index, const int particle_index, const int width, const int height, double* const data[]) :
        constraint_index(constraint_index), particle_index(particle_index), width(width), height(height), data(data) {}

    int getIndex(const int i, const int j) const { 
        return j * width + i; 
    }
};

class JacobianMatrix : public implicitMatrixWithTrans {
private:
    vector<MatrixBlock> matrix_blocks;
    int width;
    int height;
    int* constraint_indices;   // Combined indices of constraints
    int* particle_indices;     // Combined indices of particles
    const int n_dimensions = 3;

public:
    JacobianMatrix();
    virtual ~JacobianMatrix();

    int getConstraintIndex(const int i) const;
    int getParticleIndex(const int j) const;

    void initialize(const int n_particles, const int n_constraints);
    void addNewBlock(const int constraint_index, const int particle_index, const int width, const int height, double* const data[]);
    void matVecMult(double x[], double r[]) override;
    void matTransVecMult(double x[], double r[]) override;
    void vecMult(double x[], double r[], bool trans);
    void print(string name) const;
    void clear();
};

