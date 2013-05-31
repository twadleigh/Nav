package com.github.twadleigh.nav;

import org.apache.commons.math3.exception.DimensionMismatchException;
import org.apache.commons.math3.exception.NotPositiveException;
import org.apache.commons.math3.exception.OutOfRangeException;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.util.FastMath;

public class State {
	/**
	 * 
	 */
	private static final long serialVersionUID = 5465594586943090462L;
	
	// size
	private static final int SIZE = 34;
	
	// offsets
	private static final int POSITION = 0;
	private static final int VELOCITY = 3;
	private static final int ACCELERATION = 6;
	private static final int ORIENTATION = 9;
	private static final int ROTATION_RATE = 13;
	private static final int GRAVITY = 16;
	private static final int MAGNETIC_FIELD = 19;
	private static final int GPS_BIAS = 22;
	private static final int GYROSCOPE_BIAS = 25;
	private static final int ACCELEROMETER_BIAS = 28;
	private static final int MAGNETOMETER_BIAS = 31;
	
	// data
	private double[] _v = new double[SIZE];
	
	public void copyTo(State s) {
		System.arraycopy(this._v, 0, s._v, 0, SIZE);
	}
	
	public void updateState(double dt) {
		updateState(dt, _v);
	}

	static public void updateState(double dt, double st[]) {
		
		// update position
		double hdt2 = 0.5*dt*dt;
		st[POSITION+0] += dt * st[VELOCITY+0] + hdt2 * st[ACCELERATION+0];
		st[POSITION+1] += dt * st[VELOCITY+1] + hdt2 * st[ACCELERATION+1];
		st[POSITION+2] += dt * st[VELOCITY+2] + hdt2 * st[ACCELERATION+2];
		
		// update velocity
		st[VELOCITY+0] += dt * st[ACCELERATION+0];
		st[VELOCITY+1] += dt * st[ACCELERATION+1];
		st[VELOCITY+2] += dt * st[ACCELERATION+2];
		
		// *** update orientation
		
		// original quaternion
		double o0 = st[ORIENTATION+0];
		double o1 = st[ORIENTATION+1];		
		double o2 = st[ORIENTATION+2];
		double o3 = st[ORIENTATION+3];		
		
		// additional quaternion
		double omega = Math.sqrt(
				st[ROTATION_RATE+0] * st[ROTATION_RATE+0] +
				st[ROTATION_RATE+1] * st[ROTATION_RATE+1] +
				st[ROTATION_RATE+2] * st[ROTATION_RATE+2] );
		double theta = omega * dt;
		double sth = FastMath.sin(theta);
		double cth = FastMath.cos(theta);
		double alpha;
		if(theta < 1.0e-5) {
			alpha = dt;
		} else {
			alpha = sth / omega;
		}
		double q0 = cth;
		double q1 = alpha * st[ROTATION_RATE+0];
		double q2 = alpha * st[ROTATION_RATE+1];
		double q3 = alpha * st[ROTATION_RATE+2];
		
		st[ORIENTATION+0] = q0*o0 - q1*o1 - q2*o2 - q3*o3;
		st[ORIENTATION+1] = q0*o1 + q1*o0 + q2*o3 - q3*o2;
		st[ORIENTATION+2] = q0*o2 - q1*o3 + q2*o0 + q3*o1; 
		st[ORIENTATION+3] = q0*o3 + q1*o2 - q2*o1 + q3*o0; 
	}
	
	public void calcGps(double[] gps) {
		gps[0] = _v[POSITION+0] + _v[GPS_BIAS+0];
		gps[1] = _v[POSITION+1] + _v[GPS_BIAS+1];
		gps[2] = _v[POSITION+2] + _v[GPS_BIAS+2];
	}
	
	private final void rotate(double[] v) {
		// TODO define me correctly!
		double o0 = _v[ORIENTATION+0];
		double o1 = _v[ORIENTATION+1];		
		double o2 = _v[ORIENTATION+2];
		double o3 = _v[ORIENTATION+3];
		double o = Math.sqrt(o0*o0+o1*o1+o2*o2+o3*o3);
		double c = o0/o;
		double s = Math.sqrt(1.0-c*c);
		
		double x = v[0]; double y = v[1]; double z = v[2];
		v[0] = x;
		v[1] = y;
		v[2] = z;
	}
	
	public void calcAccelerometer(double[] accel) {
		accel[0] = _v[ACCELERATION+0] + _v[GRAVITY+0];
		accel[1] = _v[ACCELERATION+1] + _v[GRAVITY+1];
		accel[2] = _v[ACCELERATION+2] + _v[GRAVITY+2];
		rotate(accel);
		accel[0] += _v[ACCELEROMETER_BIAS+0];
		accel[1] += _v[ACCELEROMETER_BIAS+1];
		accel[2] += _v[ACCELEROMETER_BIAS+2];
	}
	
	public void calcGyroscope(double[] gyro) {
		gyro[0] = _v[ROTATION_RATE+0];
		gyro[1] = _v[ROTATION_RATE+1];
		gyro[2] = _v[ROTATION_RATE+2];
		rotate(gyro);
		gyro[0] += _v[GYROSCOPE_BIAS+0];
		gyro[1] += _v[GYROSCOPE_BIAS+1];
		gyro[2] += _v[GYROSCOPE_BIAS+2];
	}
	
	public void calcMagnetometer(double[] mag) {
		mag[0] = _v[MAGNETIC_FIELD+0];
		mag[1] = _v[MAGNETIC_FIELD+1];
		mag[2] = _v[MAGNETIC_FIELD+2];
		rotate(mag);
		mag[0] += _v[MAGNETOMETER_BIAS+0];
		mag[1] += _v[MAGNETOMETER_BIAS+1];
		mag[2] += _v[MAGNETOMETER_BIAS+2];
	}
	
	// Jacobians
	public void calcUpdateJacobianTranspose(double dt, double[][] J, State dState, State ws) {
		
		// TODO make this correct!
		for(int i = 0; i < SIZE; ++i) {
			double sOld = _v[i];
			double ds = dState._v[i];
			_v[i] = sOld+ds;
			updateState(J[i]);
			_v[i] = sOld-ds;
			calcGps(ws);
			_v[i] = sOld;
			J[i][0] -= ws[0];
			J[i][1] -= ws[1];
			J[i][2] -= ws[2];
			double scale = 0.5/ds;
			J[i][0] *= scale;
			J[i][1] *= scale;
			J[i][2] *= scale;
		}
	}
	
	public void calcGpsJacobianTranspose(double[][] J, State dState, double [] ws) {
		for(int i = 0; i < SIZE; ++i) {
			double sOld = _v[i];
			double ds = dState._v[i];
			_v[i] = sOld+ds;
			calcGps(J[i]);
			_v[i] = sOld-ds;
			calcGps(ws);
			_v[i] = sOld;
			J[i][0] -= ws[0];
			J[i][1] -= ws[1];
			J[i][2] -= ws[2];
			double scale = 0.5/ds;
			J[i][0] *= scale;
			J[i][1] *= scale;
			J[i][2] *= scale;
		}
	}
	
	public void calcAccelerometerJacobianTranspose(double[][] J, State dState, double [] ws) {
		for(int i = 0; i < SIZE; ++i) {
			double sOld = _v[i];
			double ds = dState._v[i];
			_v[i] = sOld+ds;
			calcAccelerometer(J[i]);
			_v[i] = sOld-ds;
			calcAccelerometer(ws);
			_v[i] = sOld;
			J[i][0] -= ws[0];
			J[i][1] -= ws[1];
			J[i][2] -= ws[2];
			double scale = 0.5/ds;
			J[i][0] *= scale;
			J[i][1] *= scale;
			J[i][2] *= scale;
		}
	}
	
	public void calcGyroscopeJacobianTranspose(double[][] J, State dState, double [] ws) {
		for(int i = 0; i < SIZE; ++i) {
			double sOld = _v[i];
			double ds = dState._v[i];
			_v[i] = sOld+ds;
			calcGyroscope(J[i]);
			_v[i] = sOld-ds;
			calcGyroscope(ws);
			_v[i] = sOld;
			J[i][0] -= ws[0];
			J[i][1] -= ws[1];
			J[i][2] -= ws[2];
			double scale = 0.5/ds;
			J[i][0] *= scale;
			J[i][1] *= scale;
			J[i][2] *= scale;
		}
	}
	
	public void calcMagnetometerJacobianTranspose(double[][] J, State dState, double [] ws) {
		for(int i = 0; i < SIZE; ++i) {
			double sOld = _v[i];
			double ds = dState._v[i];
			_v[i] = sOld+ds;
			calcMagnetometer(J[i]);
			_v[i] = sOld-ds;
			calcMagnetometer(ws);
			_v[i] = sOld;
			J[i][0] -= ws[0];
			J[i][1] -= ws[1];
			J[i][2] -= ws[2];
			double scale = 0.5/ds;
			J[i][0] *= scale;
			J[i][1] *= scale;
			J[i][2] *= scale;
		}
	}
}
