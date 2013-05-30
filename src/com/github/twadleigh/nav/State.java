package com.github.twadleigh.nav;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

public class State extends ArrayRealVector {
	/**
	 * 
	 */
	private static final long serialVersionUID = 5465594586943090462L;
	private static final int SIZE = 34;

	public State() {
		super(SIZE);
	}
	
	public State(State s) {
		super(s);
	}
	
	private final double _v(int i) { return getEntry(i); }
	
	public void getPosition(RealVector v) {
		v.setEntry(0, _v(0));
		v.setEntry(1, _v(1));
		v.setEntry(2, _v(2));
	}
	
	public void getVelocity(RealVector v) {
		v.setEntry(0, _v(3));
		v.setEntry(1, _v(4));
		v.setEntry(2, _v(5));
	}
	
	public void getAcceleration(RealVector v) {
		v.setEntry(0, _v(6));
		v.setEntry(1, _v(7));
		v.setEntry(2, _v(8));
	}
	
	public void getOrientation(RealVector v) {
		v.setEntry(0, _v(9));
		v.setEntry(1, _v(10));
		v.setEntry(2, _v(11));
		v.setEntry(3, _v(12));
	}
	
	public void getRotationRate(RealVector v) {
		v.setEntry(0, _v(13));
		v.setEntry(1, _v(14));
		v.setEntry(2, _v(15));
	}
	
	public void getGravity(RealVector v) {
		v.setEntry(0, _v(16));
		v.setEntry(1, _v(17));
		v.setEntry(2, _v(18));
	}
	
	public void getMagneticField(RealVector v) {
		v.setEntry(0, _v(19));
		v.setEntry(1, _v(20));
		v.setEntry(2, _v(21));
	}
	
	public void getGpsBias(RealVector v) {
		v.setEntry(0, _v(22));
		v.setEntry(1, _v(23));
		v.setEntry(2, _v(24));
	}
	
	public void getGyroscopeBias(RealVector v) {
		v.setEntry(0, _v(25));
		v.setEntry(1, _v(26));
		v.setEntry(2, _v(27));
	}
	
	public void getAccelerometerBias(RealVector v) {
		v.setEntry(0, _v(28));
		v.setEntry(1, _v(29));
		v.setEntry(2, _v(30));
	}
	
	public void getMagnetometerBias(RealVector v) {
		v.setEntry(0, _v(31));
		v.setEntry(1, _v(32));
		v.setEntry(2, _v(33));
	}
	
	public void calcNewState(double dt, State s) {
		
	}
	
	public void calcGps(RealVector gps, RealVector ws) {
		getPosition(gps);
		getGpsBias(ws);
		gps.combineToSelf(1.0, 1.0, ws);
	}
	
	public Array2DRowRealMatrix calcGpsJacobian(State dState) {
		Array2DRowRealMatrix J = new Array2DRowRealMatrix(3,SIZE);
		for(int i = 0; i < SIZE; ++i) {
			double sOld = this.getEntry(i);
			double ds = dState.getEntry(i);
			this.setEntry(i, sOld+ds);
			Vector3D fp = this.calcGps();
			this.setEntry(i, sOld-ds);
			Vector3D fm = this.calcGps();
			this.setEntry(i, sOld);
			Vector3D dfds = fp.subtract(fm).scalarMultiply(0.5/ds);
			J.setEntry(0, i, dfds.getX());
			J.setEntry(1, i, dfds.getY());
			J.setEntry(2, i, dfds.getZ());
		}
		return J;
	}
	
	public Vector3D calcAccelerometer() {
		return getOrientation().applyTo(getAcceleration().add(getGravity())).add(getAccelerometerBias());
	}
	
	public Vector3D calcGyroscope() {
		return getOrientation().applyTo(getRotationRate()).add(getGyroscopeBias());
	}
	
	public Vector3D calcMagnetometer() {
		return getOrientation().applyTo(getMagneticField()).add(getMagnetometerBias());
	}
	
	// Jacobians
	
}
