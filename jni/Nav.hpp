#ifndef NAV_HPP_
#define NAV_HPP_

#include <Eigen/Eigen>
#include <math.h>

typedef Eigen::Vector3d vec3;
typedef const vec3 cvec3;
typedef Eigen::Map<vec3> mvec3;
typedef Eigen::Map<cvec3> mcvec3;
typedef Eigen::Vector4d vec4;
typedef const vec4 cvec4;
typedef Eigen::Map<vec4> mvec4;
typedef Eigen::Map<cvec4> mcvec4;
typedef Eigen::AngleAxis<double> angax;
typedef Eigen::Quaternion<double> rot;
typedef const rot crot;
typedef Eigen::Map<rot> mrot;
typedef Eigen::Map<crot> mcrot;

struct LLA
{
	double lon_DEG;
	double lat_DEG;
	double alt_M;

	LLA(double lon, double lat, double alt) : lon_DEG(lon), lat_DEG(lat), alt_M(alt) {}
};

typedef const LLA cLLA;

struct Nav {

	static vec3 geodeticToEcef(cLLA&);
	static rot ecefToEnuRotation(cLLA&);

	struct State {
		static const int SIZE = 23;
		static const int POSITION_OFFSET = 0;
		static const int VELOCITY_OFFSET = 3;
		static const int ACCELERATION_OFFSET = 6;
		static const int ORIENTATION_OFFSET = 9;
		static const int ROTATION_RATE_OFFSET = 13;
		static const int GRAVITY_OFFSET = 17;
		static const int MAGNETIC_FIELD_OFFSET = 20;

		double data[SIZE];

		// mutable accessors
		mvec3 position() {
			return mvec3(data + POSITION_OFFSET);
		}
		mvec3 velocity() {
			return mvec3(data + VELOCITY_OFFSET);
		}
		mvec3 acceleration() {
			return mvec3(data + ACCELERATION_OFFSET);
		}
		mrot orientation() {
			return mrot(data + ORIENTATION_OFFSET);
		}
		double& rotationRate() {
			return data[ROTATION_RATE_OFFSET];
		}
		mvec3 rotationAxis() {
			return mvec3(data + ROTATION_RATE_OFFSET + 1);
		}
		mvec3 gravity() {
			return mvec3(data + GRAVITY_OFFSET);
		}
		mvec3 magneticField() {
			return mvec3(data + MAGNETIC_FIELD_OFFSET);
		}

		// immutable accessors
		mcvec3 position() const {
			return mcvec3(data + POSITION_OFFSET);
		}
		mcvec3 velocity() const {
			return mcvec3(data + VELOCITY_OFFSET);
		}
		mcvec3 acceleration() const {
			return mcvec3(data + ACCELERATION_OFFSET);
		}
		mcrot orientation() const {
			return mcrot(data + ORIENTATION_OFFSET);
		}
		double rotationRate() const {
			return data[ROTATION_RATE_OFFSET];
		}
		mcvec3 rotationAxis() const {
			return mcvec3(data + ROTATION_RATE_OFFSET + 1);
		}
		mcvec3 gravity() const {
			return mcvec3(data + GRAVITY_OFFSET);
		}
		mcvec3 magneticField() const {
			return mcvec3(data + MAGNETIC_FIELD_OFFSET);
		}

		// update
		void update(double dt) {

			// update position, velocity
			position() += dt * (velocity() + (0.5 * dt) * acceleration());
			velocity() += dt * acceleration();

			// normalize the rotation axis
			rotationAxis() /= rotationAxis().norm();

			// update the orientation
			orientation() = angax(dt * rotationRate(), rotationAxis())
					* orientation();
		}

		// measurement estimates
		cvec3 gps() const {
			return cvec3(position());
		}

		cvec3 accelerometer() const {
			return orientation() * (acceleration() + gravity());
		}

		cvec3 gyroscope() const {
			return rotationRate() * (orientation() * rotationAxis());
		}

		cvec3 magnetometer() const {
			return orientation() * magneticField();
		}
	};

	typedef Eigen::Matrix<double, State::SIZE, State::SIZE> covSz;
	typedef Eigen::Matrix<double, 3, 3> cov3;

	// state
	double time;
	LLA origin;
	State state;
	covSz covariance;

	// noise parameters
	covSz processNoise;
	cov3 gpsCovariance;
	cov3 accelerometerCovariance;
	cov3 gyroscopeCovariance;
	cov3 magnetometerCovariance;

	Nav(double t0, cLLA& o);
	void update(double tNew);
	void updateGps(double tNew, cLLA& gps);
	void updateGyroscope(double tNew, cvec3& gyro);
	void updateAccelerometer(double tNew, cvec3& acc);
	void updateMagnetometer(double tNew, cvec3& mag);
};

#endif /* NAV_HPP_ */
