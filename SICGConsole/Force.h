#pragma once
#include "Particle.h"
#include <vector>
#include "assert.h"
#include "GL/glut.h"
#include "MatX.h"

#define N_DIMENSIONS 3

using namespace std;

class Force {
protected:

    void printVector(Vec3 vector, string name) const {
        if (DEBUG) {
            printf("\n%s: (%.6f, %.6f, %.6f)", name.c_str(), vector[0], vector[1], vector[2]);
        }
    }

    void assertVector(Vec3 vector) const {
        for (int i = 0; i < N_DIMENSIONS; ++i) {
            assert(!isnan(vector[i]) && isfinite(vector[1]));
        }
    }
public:
    virtual ~Force() {};
    virtual void draw() const = 0;
    virtual void apply(const vector<Particle*>& pVector) = 0;
    virtual void addToJx(MatX J) = 0;
    virtual void addToJv(MatX J) = 0;
};
