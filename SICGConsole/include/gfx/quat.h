#ifndef GFXQUAT_INCLUDED // -*- C++ -*-
#define GFXQUAT_INCLUDED
#if !defined(__GNUC__)
#  pragma once
#endif

/************************************************************************

  Quaternion class
  
  $Id: quat.h 440 2005-02-23 05:14:13Z garland $

 ************************************************************************/

#include "mat4.h"

namespace gfx
{

class Quat
{
private:
    Vec3 v;			// Vector component
    double s;			// Scalar component

public:
    Quat()                                       { v=0.0; s=1.0; }
    Quat(double x, double y, double z, double w) { v[0]=x;v[1]=y;v[2]=z; s=w; }
    Quat(const Vec3& a, double b)                { v=a; s=b; }
    Quat(const Quat& q)                          { *this=q; }

    // Access methods
    const Vec3& vector() const { return v; }
    Vec3&       vector()       { return v; }
    double      scalar() const { return s; }
    double&     scalar()       { return s; }

    // Assignment and in-place arithmetic methods
    Quat& operator=(const Quat& q);
    Quat& operator+=(const Quat& q);
    Quat& operator-=(const Quat& q);
    Quat& operator=(double d);
    Quat& operator*=(double d);
    Quat& operator/=(double d);

    // Construction of standard quaternions
    static Quat ident();

    static Quat fromEuler(Vec3 euler) { // Pitch, Roll, Yaw
        Vec3 radians = euler / 180.0 * M_PI;
        double p1 = sin(radians[2] / 2.0); // Yaw
        double p2 = cos(radians[2] / 2.0); // Yaw
        double r1 = sin(radians[0] / 2.0); // Pitch
        double r2 = cos(radians[0] / 2.0); // Pitch
        double y1 = sin(radians[1] / 2.0); // Roll
        double y2 = cos(radians[1] / 2.0); // Roll

        return Quat(
            r1 * p2 * y2 - r2 * p1 * y1,
            r2 * p1 *y2 + r1 * p2 * y1,
            r2 * p2 * y1 - r1 * p1 * y2,
            r2 * p2 * y2 + r1 * p1 * y1
        );
    }

    void print() {
        printf("x: %.2f, y: %.2f, z: %.2f, w: %.2f\n", v[0], v[1], v[2], s);
    }
};

////////////////////////////////////////////////////////////////////////
//
// Implementation of Quat methods
//

inline Quat& Quat::operator=(const Quat& q) { v=q.v; s=q.s; return *this; }
inline Quat& Quat::operator+=(const Quat& q) { v+=q.v; s+=q.s; return *this; }
inline Quat& Quat::operator-=(const Quat& q) { v-=q.v; s-=q.s; return *this; }

inline Quat& Quat::operator=(double d)  { v=d;  s=d;  return *this; }
inline Quat& Quat::operator*=(double d) { v*=d; s*=d; return *this; }
inline Quat& Quat::operator/=(double d) { v/=d; s/=d; return *this; }

inline Quat Quat::ident() { return Quat(0, 0, 0, 1); }

////////////////////////////////////////////////////////////////////////
//
// Standard arithmetic operators on quaternions
//

inline Quat operator+(const Quat& q, const Quat& r)
	{ return Quat(q.vector()+r.vector(), q.scalar()+r.scalar()); }

inline Quat operator*(const Quat& q, const Quat& r)
{
    return Quat(cross(q.vector(),r.vector()) +
		r.scalar()*q.vector() +
		q.scalar()*r.vector(),
		q.scalar()*r.scalar() - q.vector()*r.vector());
}

inline Quat operator*(const Quat& q, double s)
	{ return Quat(q.vector()*s, q.scalar()*s); }
inline Quat operator*(double s, const Quat& q)
	{ return Quat(q.vector()*s, q.scalar()*s); }

inline Quat operator/(const Quat& q, double s)
	{ return Quat(q.vector()/s, q.scalar()/s); }

inline std::ostream &operator<<(std::ostream &out, const Quat& q)
	{ return out << q.vector() << " " << q.scalar(); }

inline std::istream &operator>>(std::istream &in, Quat& q)
	{ return in >> q.vector() >> q.scalar(); }


////////////////////////////////////////////////////////////////////////
//
// Standard functions on quaternions
//

inline double norm(const Quat& q)
{ 
    return q.scalar()*q.scalar() + q.vector()*q.vector(); 
}

inline Quat conjugate(const Quat& q) { return Quat(-q.vector(), q.scalar()); }
inline Quat inverse(const Quat& q) { return conjugate(q)/norm(q); }
inline Quat& unitize(Quat& q)  { q /= sqrt(norm(q)); return q; }

extern Quat exp(const Quat& q);
extern Quat log(const Quat& q);
extern Quat axis_to_quat(const Vec3& a, double phi);
inline Mat3 quat_to_matrix(const Quat& q) {
    Vec3 v = q.vector();
    double s = q.scalar();
    Vec3 pv = Vec3(pow(v[0], 2), pow(v[1], 2), pow(v[2], 2));
    return Mat3(
        Vec3(1.0 - 2.0*pv[1] - 2.0*pv[2]    , 2.0*v[0]*v[1] - 2.0*s*v[2]    , 2.0*v[0]*v[2] + 2.0*s*v[1]),
        Vec3(2.0*v[0]*v[1] + 2.0*s*v[2]     , 1.0 - 2.0*pv[0] - 2.0*pv[2]   , 2.0*v[1]*v[2] - 2.0*s*v[0]),
        Vec3(2.0*v[0]*v[2] - 2.0*s*v[1]     , 2.0*v[1]*v[2] + 2.0*s*v[0]    , 1.0 - 2.0*pv[0] - 2.0*pv[1])
    );
}
extern Mat4 unit_quat_to_matrix(const Quat& q);
extern Quat slerp(const Quat& from, const Quat& to, double t);

} // namespace gfx

// GFXQUAT_INCLUDED
#endif
