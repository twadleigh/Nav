package com.github.twadleigh.nav;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;

import android.location.Location;

public class Nav {
	
	// *** static stuff ***

	// WGS84 constants
	static final public double WGS84_SEMI_MAJOR_AXIS_M = 6378137.0;
	static final public double WGS84_INVERSE_FLATTENING = 298.257223563;
	static final public double WGS84_FLATTENING = 1.0/Nav.WGS84_INVERSE_FLATTENING;
	static final public double WGS84_FIRST_ECCENTRICITY_SQUARED = 
			1.0-(1.0-Nav.WGS84_FLATTENING)*(1.0-Nav.WGS84_FLATTENING);
	
	static public Vector3D geodeticToEcef(double lon_DEG, double lat_DEG, double alt_M) {
		double lon_RAD = Math.toRadians(lon_DEG);
		double lat_RAD = Math.toRadians(lat_DEG);
		double sLon = Math.sin(lon_RAD);
		double cLon = Math.cos(lon_RAD);
		double sLat = Math.sin(lat_RAD);
		double cLat = Math.cos(lat_RAD);
		
		double N = Nav.WGS84_SEMI_MAJOR_AXIS_M/
				Math.sqrt(1.0-Nav.WGS84_FIRST_ECCENTRICITY_SQUARED*sLat*sLat);
		
		return new Vector3D(
				(N+alt_M)*cLat*cLon,
				(N+alt_M)*cLat*sLon,
				(N*(1.0-Nav.WGS84_FIRST_ECCENTRICITY_SQUARED)+alt_M)*sLat);
	}
	
	static public Rotation ecefToEnuRotation(double lon_DEG, double lat_DEG, double alt_M) {
		return new Rotation(
				RotationOrder.ZXZ, 
				Math.toRadians(lon_DEG), 
				Math.toRadians(lat_DEG),
				0.0);
	}
	
	// *** state ***
	
	// origin of ENU frame
	private Vector3D origin_LLA;
	
	// time (nanoseconds since startup)
	private long t_nS;
	
	// position
	private Vector3D x_M;
	
	// velocity
	private Vector3D v_MpS;
	
	// acceleration
	private Vector3D a_MpS2;
	
	// orientation
	private Rotation theta;
	
	// rotation rate
	private Vector3D omega_RADpS;
	
	// gravity
	private Vector3D g_MpS2;
	
	// magnetic field
	private Vector3D f_uT;
	
	// GPS bias
	private Vector3D x0_M;
	
	// accelerometer bias
	private Vector3D a0_MpS2;
	
	// gyroscope bias
	private Vector3D omega0_RADpS;
	
	// magnetometer bias
	private Vector3D f0_uT;
	
	// system error
	private RealMatrix P;
	
	// process error
	// TODO implement this!
	private final RealMatrix Q = null;
	
	void update(long  tNew_nS) {
		if(tNew_nS == t_nS) return;
		
		double dt_S = 1.0e-9*((double)(tNew_nS-t_nS));
		
		// update time
		t_nS = tNew_nS;
		
		// update position
		x_M = x_M.add(dt_S, v_MpS).add(0.5*dt_S*dt_S, a_MpS2);
		
		// update velocity
		v_MpS = v_MpS.add(dt_S, a_MpS2);
		
		// update orientation
		double dTheta_RAD = dt_S * omega_RADpS.getNorm();
		theta = (new Rotation(omega_RADpS,dTheta_RAD)).applyTo(theta);
		
		// update error
		RealMatrix F = getJacobian();
		P = F.multiply(P).multiply(F.transpose()).add(Q.scalarMultiply(dt_S));
	}
	
	RealMatrix getJacobian() {
		// TODO implement this!
		return null;
	}
	
	void fuseGps(long timestamp, double lon_DEG, double lat_DEG, double alt_M, double acc_M) {
		// TODO implement this!
	}
}
