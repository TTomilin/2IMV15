#pragma once

#include "Particle.h"
#include "Constraint.h"
#include "JacobianMatrix.h"

class CircularWireConstraint : public Constraint {
 public:
  CircularWireConstraint(Particle *p, const Vec3 & center, const double radius, int index);

  void draw() const override;
  double getC() override;
  double getCDot() override;
  vector<Vector3f> getJ() override;
  vector<Vector3f> getJDot() override;
  void addToJx(MatX Jx) override;
  void addToJv(MatX Jv) override;

 private:
  Particle * const m_p;
  Vec3 const m_center;
  double const m_radius;
  double const m_radiusSquared;
};
