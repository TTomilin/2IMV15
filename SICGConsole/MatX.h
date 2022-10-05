#pragma once
#include "linearSolver.h"
#include "include/gfx/mat3.h"
#include <vector>

using namespace std;

class MatX : public implicitMatrix
{
public:
    const int DIMS = 3;
    const int m_size;

    std::vector<std::vector<double>> values;

    MatX(const int size, const double value);
    MatX(const int size);
    MatX(const MatX& mat);

    void matVecMult(double x[], double r[]);
    double* mult(double x[]);
    MatX addDiag(double value);
    MatX addToSection(int index_x, int index_y, Mat3& derivs);
    MatX add(const MatX mat);
    MatX sub(const MatX mat);
    MatX mult(const double mult);
    MatX div(const double div);
    MatX clone();
};