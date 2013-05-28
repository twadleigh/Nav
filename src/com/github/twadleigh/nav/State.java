package com.github.twadleigh.nav;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.linear.ArrayRealVector;

public class State extends ArrayRealVector {
	/**
	 * 
	 */
	private static final long serialVersionUID = 5465594586943090462L;

	public State() {
		super(34);
	}
	
	public State(State s) {
		super(s);
	}
	
	private final double _v(int i) { return getEntry(i); }
	
	public Vector3D getPosition() {
		return new Vector3D(_v(0),_v(1),_v(2));
	}
	
	public Vector3D getVelocity() {
		return new Vector3D(_v(3),_v(4),_v(5));
	}
	
	public Vector3D getAcceleration() {
		return new Vector3D(_v(6),_v(7),_v(8));
	}
	
	public Rotation getOrientation() {
		return new Rotation(_v(9),_v(10),_v(11),_v(12),true);
	}
	
	public Vector3D getRotationRate() {
		return new Vector3D(_v(13),_v(14),_v(15));
	}
	
	public Vector3D getGravity() {
		return new Vector3D(_v(16),_v(17),_v(18));
	}
	
	public Vector3D getMagneticField() {
		return new Vector3D(_v(19),_v(20),_v(21));
	}
	
	public Vector3D getGpsBias() {
		return new Vector3D(_v(22),_v(23),_v(24));
	}
	
	public Vector3D getGyroscopeBias() {
		return new Vector3D(_v(25),_v(26),_v(27));
	}
	
	public Vector3D getAccelerometerBias() {
		return new Vector3D(_v(28),_v(29),_v(30));
	}
	
	public Vector3D getMagnetometerBias() {
		return new Vector3D(_v(31),_v(32),_v(33));
	}
	
	public void calcNewState(double dt, State s) {
		
	}
	
	public Vector3D calcGps() {
		return getPosition().add(getGpsBias());
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
}
