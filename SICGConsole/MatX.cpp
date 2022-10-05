#include "stdafx.h"
#include "MatX.h"
#include "assert.h"

MatX::MatX(const MatX& mat) : m_size(mat.m_size) {
	values = std::vector<std::vector<double>>(mat.values);
}

// Initialize matrix of certain size filled entirely with certain value
MatX::MatX(const int size, const double value) : m_size(size) {
	values = std::vector<std::vector<double>>();
	for (int i = 0; i < m_size; i++) {
		std::vector<double> row (m_size);
		fill(row.begin(), row.end(), value);
		values.push_back(row);
	}
}

// Initialize matrix of certain size (entirely filled with 0's)
MatX::MatX(const int size) : MatX(size, 0.) {
}

// Calculate r s.t. MatX * x = r
void MatX::matVecMult(double x[], double r[]) {
	for (int i = 0; i < m_size; i++) {
		vector<double> row = values.at(i);
		r[i] = 0.;
		for (int j = 0; j < m_size; j++) {
			double value = row.at(j);
			r[i] += value * x[i];
		}
	}
}

double* MatX::mult(double x[]) {
	double* r = new double[m_size];
	matVecMult(x, r);
	return r;
}

// Add a section for the derivative of particle X w.r.t. particle Y
MatX MatX::addToSection(int index_x, int index_y, Mat3& derivs) {
	for (int i = 0; i < DIMS; i++) {
		for (int j = 0; j < DIMS; j++) {
			values[index_x * DIMS + i][index_y * DIMS + j] += derivs[i][j];
		}
	}
	return *this;
}

// Add a certain value to the diagonal of this matrix
MatX MatX::addDiag(double value) {
	for (int i = 0; i < m_size; i++) {
		values.at(i).at(i) += value;
	}
	return *this;
}

// Add another MatX's values to this matrix's
MatX MatX::add(const MatX add) {
	assert(m_size == add.m_size);
	for (int i = 0; i < m_size; i++) {
		vector<double> rowThis = values.at(i);
		vector<double> rowAdd = add.values.at(i);
		for (int j = 0; j < m_size; j++) {
			rowThis.at(j) += rowAdd.at(j);
		}
	}
	return *this;
}

// Subtract another MatX's values from this matrix's
MatX MatX::sub(const MatX mat) {
	// A - B = -(-A+B)
	mult(-1);
	add(mat);
	mult(-1);
	return *this;
}

// Multiply all values in this matrix with this value
MatX MatX::mult(const double mult) {
	for (int i = 0; i < m_size; i++) {
		vector<double> row = values.at(i);
		for (int j = 0; j < m_size; j++) {
			row.at(j) *= mult;
		}
	}
	return *this;
}

// Clone the matrix into a new instance
MatX MatX::clone() {
	return MatX(*this);
}

// Divide all values in this matrix with this value
MatX MatX::div(const double div) {
	for (int i = 0; i < m_size; i++) {
		vector<double> row = values.at(i);
		for (int j = 0; j < m_size; j++) {
			row.at(j) /= div;
		}
	}
	return *this;
}
