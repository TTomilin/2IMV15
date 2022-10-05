#pragma once

#include "Particle.h"
#include "Constraint.h"
#include "JacobianMatrix.h"

class RodConstraint : public Constraint {
 public:
  RodConstraint(Particle* p1, Particle* p2, const double dist, const int id);

  void draw() const override;
  double getC() override;
  double getCDot() override;
  vector<Vector3f> getJ() override;
  vector<Vector3f> getJDot() override;
  void addToJx(MatX Jx) override;
  void addToJv(MatX Jv) override;

 private:
  const Particle * const m_p1;
  const Particle * const m_p2;
  double const m_dist_squared;
};
