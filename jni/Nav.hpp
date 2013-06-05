#ifndef NAV_HPP_
#define NAV_HPP_

#include<Eigen/Eigen>
#include<Eigen/Geometry>

struct State {
	static int SIZE = 23;
	static int POSITION_OFFSET = 0;
	static int VELOCITY_OFFSET = 3;
	static int ACCELERATION_OFFSET = 6;
	static int ORIENTATION_OFFSET = 9;
	static int ROTATION_RATE_OFFSET = 13;
	static int GRAVITY_OFFSET = 17;
	static int MAGNETIC_FIELD_OFFSET = 20;

	double data[SIZE];

	typedef Eigen::Vector3d vec;
	typedef const vec cvec;
	typedef Eigen::Map<vec> mvec;
	typedef Eigen::Map<cvec> mcvec;
	typedef Eigen::Quaternion<double> rot;
	typedef const rot crot;
	typedef Eigen::Map<rot> mrot;
	typedef Eigen::Map<crot> mcrot;
	typedef Eigen::AngleAxis<double> angax;

	// mutable accessors
	mvec position()        { return mvec(data+POSITION_OFFSET); }
	mvec velocity()        { return mvec(data+VELOCITY_OFFSET); }
	mvec acceleration()    { return mvec(data+ACCELERATION_OFFSET); }
	mrot orientation()     { return mrot(data+ORIENTATION_OFFSET); }
	double& rotationRate() { return data[ROTATION_RATE_OFFSET]; }
	mvec rotationAxis()    { return mrot(data+ROTATION_RATE_OFFSET+1); }
	mvec gravity()         { return mvec(data+GRAVITY_OFFSET); }
	mvec magneticField()   { return mvec(data+MAGNETIC_FIELD_OFFSET); }

	// immutable accessors
	mcvec position()      const { return mcvec(data+POSITION_OFFSET); }
	mcvec velocity()      const { return mcvec(data+VELOCITY_OFFSET); }
	mcvec acceleration()  const { return mcvec(data+ACCELERATION_OFFSET); }
	mcrot orientation()   const { return mcrot(data+ORIENTATION_OFFSET); }
	double rotationRate() const { return data[ROTATION_RATE_OFFSET]; }
	mcvec rotationAxis()  const { return mcrot(data+ROTATION_RATE_OFFSET+1); }
	mcvec gravity()       const { return mcvec(data+GRAVITY_OFFSET); }
	mcvec magneticField() const { return mcvec(data+MAGNETIC_FIELD_OFFSET); }

	// update
	void update(double dt) {

		// update position, velocity
		position() += dt * (velocity() + (0.5*dt) * acceleration());
		velocity() += dt * acceleration();

		// normalize the rotation axis
		rotationAxis() /= rotationAxis().norm();

		// update the orientation
		orientation() = angax(dt * rotationRate(), rotationAxis()) * orientation();
	}

	// measurement estimates
	vec gps() const {
		return vec(position());
	}

	vec accelerometer() const {
		return orientation() * (acceleration()+gravity());
	}

	vec gyroscope() const {
		return rotationRate() * (orientation() * rotationAxis());
	}

	vec magnetometer() const {
		return orientation() * magneticField();
	}
};

#endif /* NAV_HPP_ */
