/*
 * Nav.hpp
 *
 *  Created on: Jun 1, 2013
 *      Author: tracy
 */

#ifndef NAV_HPP_
#define NAV_HPP_

#include <math.h>

struct Vec3 {
	double x;
	double y;
	double z;

	Vec3(double _x = 0.0, double _y = 0.0, double _z = 0.0) : x(_x), y(_y), z(_z) {}
	Vec3(const Vec3& v) : x(v.x), y(v.y), z(v.z) {}

	Vec3& operator+=(const Vec3& v) { x += v.x; y += v.y; z += v.z; return *this; }
	Vec3& operator+=(double s)      { x += s;   y += s;   z += s;   return *this; }
	Vec3& operator-=(const Vec3& v) { x -= v.x; y -= v.y; z -= v.z; return *this; }
	Vec3& operator-=(double s)      { x -= s;   y -= s;   z -= s;   return *this; }
	Vec3& operator*=(const Vec3& v) { x *= v.x; y *= v.y; z *= v.z; return *this; }
	Vec3& operator*=(double s)      { x *= s;   y *= s;   z *= s;   return *this; }
	Vec3& operator/=(const Vec3& v) { x /= v.x; y /= v.y; z /= v.z; return *this; }
	Vec3& operator/=(double s)      { x /= s;   y /= s;   z /= s;   return *this; }

	Vec3 operator+(const Vec3& v) const { return Vec3(*this) += v; }
	Vec3 operator+(double s)      const { return Vec3(*this) += s; }
	Vec3 operator-(const Vec3& v) const { return Vec3(*this) -= v; }
	Vec3 operator-(double s)      const { return Vec3(*this) -= s; }
	Vec3 operator*(const Vec3& v) const { return Vec3(*this) *= v; }
	Vec3 operator*(double s)      const { return Vec3(*this) *= s; }
	Vec3 operator/(const Vec3& v) const { return Vec3(*this) /= v; }
	Vec3 operator/(double s)      const { return Vec3(*this) /= s; }

	Vec3 operator-() const { return Vec3(-x,-y,-z); }

	double dot(const Vec3& v) const { return x*v.x+y*v.y+z*v.z; }
	double norm2()            const { return dot(*this); }
	double norm()             const { return sqrt(norm2()); }
};

struct Quat {
	double r;
	Vec3 i;

	Quat(double _r = 0.0, double _i = 0.0, double _j = 0.0, double _k = 0.0) : r(_r), i(_i,_j,_k) {}
	Quat(double _r, const Vec3& _i) : r(_r), i(_i) {}
	Quat(const Vec3& _i) : r(0.0), i(_i) {}

	Quat& operator+=(const Quat& q) { r += q.r; i += q.i; return *this; }
	Quat& operator+=(double s)      { r += s;   i += s;   return *this; }
	Quat& operator-=(const Quat& q) { r -= q.r; i -= q.i; return *this; }
	Quat& operator-=(double s)      { r -= s;   i -= s;   return *this; }

	Quat operator+(const Quat& v) const { return Quat(*this) += v; }
	Quat operator+(double s)      const { return Quat(*this) += s; }
	Quat operator-(const Quat& v) const { return Quat(*this) -= v; }
	Quat operator-(double s)      const { return Quat(*this) -= s; }

	Quat conj() const { return Quat(r,-i); }

	Quat operator*(const Quat& q) const {
		return Quat(
				r*q.r   - i.x*q.i.x - i.y*q.i.y - i.z*q.i.z,
				r*q.i.x + i.x*q.r   + i.y*q.i.z - i.z*q.i.y,
				r*q.i.y - i.x*q.i.z + i.y*q.r   + i.z*q.i.x,
				r*q.i.z + i.x*q.i.y - i.y*q.i.x + i.z*q.r );
	}

	Vec3 rotate(const Vec3& v) const {
		return ((*this) * Quat(v) * (*this).conj()).i / norm2();
	}

	double dot(const Quat& q) const { return r*q.r+i.dot(q.i); }
	double norm2()            const { return dot(*this); }
	double norm()             const { return sqrt(norm2()); }
};

struct State {
	Vec3 x;			// position
	Vec3 v;			// velocity
	Vec3 a;			// acceleration
	Quat theta;		// orientation
	Vec3 omega;		// rotation rate
	Vec3 g;			// gravity
	Vec3 f;			// magnetic field

	State(const State s) : x(s.x), v(s.v), a(s.a), theta(s.theta), omega(s.omega), g(s.g), f(s.f) {}

	// update
	void update(double dt) {
		x += dt * (v + (0.5*dt) * a);
		v += dt * a;
		double omga = omega.norm();
		double hdtheta = 0.5 * dt * omga;
		double alpha = (omga < 1.0e-5) ? 0.5 * dt : sin(hdtheta) / omga;
		theta = Quat(cos(hdtheta), alpha * omega) * theta;
	}

	// measurement estimates
	Vec3 gps() const {
		return x;
	}

	Vec3 accelerometer() const {
		return theta.rotate(a+g);
	}

	Vec3 gyroscope() const {
		return theta.rotate(omega);
	}

	Vec3 magnetometer() const {
		return theta.rotate(f);
	}
};

struct Nav {

};

#endif /* NAV_HPP_ */
