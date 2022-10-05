#ifndef GFXMAT3_INCLUDED // -*- C++ -*-
#define GFXMAT3_INCLUDED
#if !defined(__GNUC__)
#  pragma once
#endif

/************************************************************************

  3x3 Matrix class

  $Id: mat3.h 427 2004-09-27 04:45:31Z garland $

 ************************************************************************/

#include "vec3.h"
#include "mat2.h"

namespace gfx
{

class Mat3
{
private:
    Vec3 row[3];

public:
    // Standard constructors
    //
    Mat3() { *this = 0.0; }
    Mat3(const Vec3& r0,const Vec3& r1,const Vec3& r2)
    	{ row[0]=r0; row[1]=r1; row[2]=r2; }
    Mat3(const Mat3& m) { *this = m; }

    // Descriptive interface
    //
    typedef double value_type;
    typedef Vec3 vector_type;
    typedef Mat3 inverse_type;
    static int dim() { return 3; }

    // Access methods
    // 
    double& operator()(int i, int j)       { return row[i][j]; }
    double  operator()(int i, int j) const { return row[i][j]; }
    Vec3&       operator[](int i)       { return row[i]; }
    const Vec3& operator[](int i) const { return row[i]; }
    inline Vec3 col(int i) const {return Vec3(row[0][i],row[1][i],row[2][i]);}

    operator       double*()       { return row[0]; }
    operator const double*()       { return row[0]; }
    operator const double*() const { return row[0]; }


    // Assignment methods
    //
    inline Mat3& operator=(const Mat3& m);
    inline Mat3& operator=(double s);

    inline Mat3& operator+=(const Mat3& m);
    inline Mat3& operator-=(const Mat3& m);
    inline Mat3& operator*=(double s);
    inline Mat3& operator/=(double s);


    // Construction of standard matrices
    //
    static Mat3 I() {
        return Mat3(
            Vec3(1., 0., 0.),
            Vec3(0., 1., 0.),
            Vec3(0., 0., 1.)
        );
    };
    static Mat3 outer_product(const Vec3& a, const Vec3& b) {
        return Mat3(
            Vec3(a[0] * b[0], a[0] * b[1], a[0] * b[2]),
            Vec3(a[1] * b[0], a[1] * b[1], a[1] * b[2]),
            Vec3(a[2] * b[0], a[2] * b[1], a[2] * b[2])
        );
    }
    static Mat3 outer_product(const Vec3& v) {
        return outer_product(v, v);
    }

    Mat3 &diag(double d);
    Mat3 &ident() { return diag(1.0); }

    Mat3 inverse() {
        double determinant = row[0] * (row[1] ^ row[2]);
        Mat3 res = Mat3(
            Vec3(
                det(Mat2(Vec2(row[1][1], row[1][2]), Vec2(row[2][1], row[2][2]))),
                det(Mat2(Vec2(row[0][2], row[0][1]), Vec2(row[2][2], row[2][1]))),
                det(Mat2(Vec2(row[0][1], row[0][2]), Vec2(row[1][1], row[1][2])))
            ),
            Vec3(
                det(Mat2(Vec2(row[1][2], row[1][0]), Vec2(row[2][2], row[2][0]))),
                det(Mat2(Vec2(row[0][0], row[0][2]), Vec2(row[2][0], row[2][2]))),
                det(Mat2(Vec2(row[0][2], row[0][0]), Vec2(row[1][2], row[1][0])))
            ),
            Vec3(
                det(Mat2(Vec2(row[1][0], row[1][1]), Vec2(row[2][0], row[2][1]))),
                det(Mat2(Vec2(row[0][1], row[0][0]), Vec2(row[2][1], row[2][0]))),
                det(Mat2(Vec2(row[0][0], row[0][1]), Vec2(row[1][0], row[1][1])))
            )
        );
        res[0] = res[0] * (1.0 / determinant);
        res[1] = res[1] * (1.0 / determinant);
        res[2] = res[2] * (1.0 / determinant);
        return res;
    }

    void print() {
        printf("%.4f, %.4f, %.4f\n%.4f, %.4f, %.4f\n%.4f, %.4f, %.4f\n", 
            row[0][0], row[0][1], row[0][2],
            row[1][0], row[1][1], row[1][2],
            row[2][0], row[2][1], row[2][2]
        );
    }

};

////////////////////////////////////////////////////////////////////////
//
// Methods definitions
//

inline Mat3& Mat3::operator=(const Mat3& m)
	{ row[0] = m[0]; row[1] = m[1]; row[2] = m[2];  return *this; }

inline Mat3& Mat3::operator=(double s)
	{ row[0]=s;  row[1]=s;  row[2]=s;  return *this; }

inline Mat3& Mat3::operator+=(const Mat3& m)
	{ row[0] += m[0]; row[1] += m[1]; row[2] += m[2]; return *this; }

inline Mat3& Mat3::operator-=(const Mat3& m)
	{ row[0] -= m[0]; row[1] -= m[1]; row[2] -= m[2]; return *this; }

inline Mat3& Mat3::operator*=(double s)
	{ row[0] *= s; row[1] *= s; row[2] *= s;  return *this; }

inline Mat3& Mat3::operator/=(double s)
	{ row[0] /= s; row[1] /= s; row[2] /= s;  return *this; }

////////////////////////////////////////////////////////////////////////
//
// Operator definitions
//

inline Mat3 operator+(const Mat3& n, const Mat3& m)
	{ return Mat3(n[0]+m[0], n[1]+m[1], n[2]+m[2]); }

inline Mat3 operator-(const Mat3& n, const Mat3& m)
	{ return Mat3(n[0]-m[0], n[1]-m[1], n[2]-m[2]); }

inline Mat3 operator-(const Mat3& m)
	{ return Mat3(-m[0], -m[1], -m[2]); }

inline Mat3 operator*(double s, const Mat3& m)
	{ return Mat3(m[0]*s, m[1]*s, m[2]*s); }
inline Mat3 operator*(const Mat3& m, double s)
	{ return s*m; }

inline Mat3 operator/(const Mat3& m, double s)
	{ return Mat3(m[0]/s, m[1]/s, m[2]/s); }

inline Vec3 operator*(const Mat3& m, const Vec3& v)
	{ return Vec3(m[0]*v, m[1]*v, m[2]*v); }

inline Mat3 operator*(const Mat3& n, const Mat3& m) {
    Mat3 res = Mat3(Vec3(0, 0, 0), Vec3(0, 0, 0), Vec3(0, 0, 0));
    for (uint8_t x = 0; x < 3; x++) {
        for (uint8_t y = 0; y < 3; y++) {
            res[y][x] = m[y][0] * n[0][x] + m[y][1] * n[1][x] + m[y][2] * n[2][x];
        }
    }
    return res;
}

inline std::ostream &operator<<(std::ostream &out, const Mat3& M)
	{ return out << M[0] << std::endl << M[1] << std::endl << M[2]; }

inline std::istream &operator>>(std::istream &in, Mat3& M)
	{ return in >> M[0] >> M[1] >> M[2]; }

////////////////////////////////////////////////////////////////////////
//
// Misc. function definitions
//

inline double det(const Mat3& m) { return m[0] * (m[1] ^ m[2]); }

inline double trace(const Mat3& m) { return m(0,0) + m(1,1) + m(2,2); }

inline Mat3 transpose(const Mat3& m)
	{ return Mat3(m.col(0), m.col(1), m.col(2)); }
	
extern Mat3 adjoint(const Mat3& m);

extern double invert(Mat3& m_inv, const Mat3& m);

inline Mat3 row_extend(const Vec3& v) { return Mat3(v, v, v); }

extern Mat3 diag(const Vec3& v);

extern bool eigen(const Mat3& m, Vec3& eig_vals, Vec3 eig_vecs[3]);

} // namespace gfx

// GFXMAT3_INCLUDED
#endif
