#include "stdafx.h"
#include "JacobianMatrix.h"
#include <assert.h>
#include <iostream>

JacobianMatrix::JacobianMatrix() {}
JacobianMatrix::~JacobianMatrix() {}

/**
 * This method has to be called to set up the matrix.
 * Unable to instantiate static field variable during runtime.
 */
void JacobianMatrix::initialize(const int n_particles, const int n_constraints) {
    int* width = new int[n_constraints];
    int* height = new int[n_particles];

    fill(width, width + n_constraints, 0);
    fill(height, height + n_particles, n_dimensions);

    int i;
    for (i = 0; i < matrix_blocks.size(); ++i) {
        width[matrix_blocks[i].constraint_index] = matrix_blocks[i].width;
    }

    delete[] particle_indices;
    delete[] constraint_indices;

    particle_indices = new int[n_particles];
    constraint_indices = new int[n_constraints];

    if (n_particles > 0) { 
        particle_indices[0] = 0; 
    }
    if (n_constraints > 0) { 
        constraint_indices[0] = 0; 
    }

    for (i = 1; i < n_constraints; ++i) { 
        constraint_indices[i] = constraint_indices[i - 1] + width[i - 1]; 
    }

    for (i = 1; i < n_particles; ++i) { 
        particle_indices[i] = particle_indices[i - 1] + height[i - 1]; 
    }

    this->width = n_constraints == 0 ? 0 : constraint_indices[n_constraints - 1] + width[n_constraints - 1];
    this->height = n_particles == 0 ? 0 : particle_indices[n_particles - 1] + height[n_particles - 1];
}

void JacobianMatrix::matVecMult(double x[], double r[]) {
    vecMult(x, r, false);
}

void JacobianMatrix::matTransVecMult(double x[], double r[]) {
    vecMult(x, r, true);
}

void JacobianMatrix::vecMult(double x[], double r[], bool trans) {
    int k, i, j;
    for (k = 0; k < width; k++) {
        r[k] = 0.0;
    }

    // Iterate every block within this matrix
    for (k = 0; k < matrix_blocks.size(); ++k) {
        MatrixBlock block = matrix_blocks[k];

        // Iterate every constraint within this block
        for (i = 0; i < block.width; ++i) {
            int constraint_index = getConstraintIndex(block.constraint_index) + i;

            // Iterate every particle within this constraint
            for (j = 0; j < block.height; ++j) {
                int particle_index = getParticleIndex(block.particle_index) + j;    // Get the index of the particle
                double value = *(block.data[block.getIndex(i, j)]);                 // Get the value from the block by the given indices
                if (DEBUG) printf("\nBlock data at (%d, %d): %.6f", i, j, value);

                if (trans) {
                    r[particle_index] += value * x[constraint_index];               // Increase the scaled constraint value for this particle
                    if (DEBUG) {
                        printf("\nX at %d: %.6f", constraint_index, x[constraint_index]);
                        printf("\nNew R at %d: %.6f", particle_index, r[particle_index]);
                    }
                    assert(!isnan(r[particle_index]) && isfinite(r[particle_index]));
                }

                else {
                    r[constraint_index] += value * x[particle_index];               // Increase the scaled particle value for this constraint
                    if (DEBUG) {
                        printf("\nX at %d: %.6f", particle_index, x[particle_index]);
                        printf("\nNew R at %d: %.6f", constraint_index, r[constraint_index]);
                    }
                    assert(!isnan(r[constraint_index]) && isfinite(r[constraint_index]));
                }
            }
        }
    }
}

void JacobianMatrix::addNewBlock(const int constraint_index, const int particle_index, const int width, const int height, double* const data[]) {
    matrix_blocks.push_back(MatrixBlock(constraint_index, particle_index, width, height, data));
}

int JacobianMatrix::getConstraintIndex(const int i) const {
    return constraint_indices[i];
}

int JacobianMatrix::getParticleIndex(const int j) const {
    return particle_indices[j];
}

void JacobianMatrix::print(string name) const {
    if (!DEBUG) {
        return;
    }
    vector<vector<double>> matrix;
    int i, j, k;

    for (int i = 0; i < width; i++) {
        vector<double> row;
        for (int j = 0; j < height; j++) {
            row.push_back(0);
        }
        matrix.push_back(row);
    }

    for (k = 0; k < matrix_blocks.size(); ++k) {
        for (i = 0; i < matrix_blocks[k].width; i++) {
            int index_i = matrix_blocks[k].constraint_index + i;
            for (j = 0; j < matrix_blocks[k].height; j++) {
                int index_j = matrix_blocks[k].particle_index * 2 + j;
                matrix[index_i][index_j] = *(matrix_blocks[k].data[matrix_blocks[k].getIndex(i, j)]);
            }
        }
    }

    cout << endl << name << " (" << width << "," << height << ")" << endl;
    for (j = 0; j < height; j++) {
        for (i = 0; i < width; i++) {
            cout << matrix[i][j] << "  ";
        }
        cout << endl;
    }
}

void JacobianMatrix::clear() {
    matrix_blocks.clear();
    delete[] constraint_indices;
    delete[] particle_indices;
    constraint_indices = nullptr;
    particle_indices = nullptr;
    width = 0;
    width = 0;
}
