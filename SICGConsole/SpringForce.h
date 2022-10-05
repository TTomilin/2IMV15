#pragma once

#include "Particle.h"
#include "Force.h"


/*
ks -> spring strenght
kd -> spring dampening
*/
class SpringForce : public Force {
 public:
  SpringForce(Particle *p1, Particle * p2, double dist, double ks, double kd);
  
  void draw() const override;
  void apply(const std::vector<Particle*>& particles) override;
  void addToJx(MatX Jx) override;
  void addToJv(MatX Jv) override;

 private:

  Particle * const m_p1;   // particle 1
  Particle * const m_p2;   // particle 2 
  double const m_dist;     // rest length
  double const m_ks, m_kd; // spring strength constants
};
