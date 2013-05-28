package com.github.twadleigh.nav;

import java.util.Arrays;

import org.apache.commons.math3.complex.Quaternion;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

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
	
	// state estimate
	private State x = new State();
	private RealMatrix P = new Array2DRowRealMatrix(34,34);
	
	// parameters
	private static State x0; // initial state
	private static State P0; // initial uncertainty
	private static RealMatrix Q; // process error
	private static RealMatrix Rgps; // GPS noise
	private static RealMatrix Racc; // accelerometer noise
	private static RealMatrix Rgyr; // gyroscope noise
	private static RealMatrix Rmag; // magnetometer noise
	
	public Nav() {
		 x = x0.copy();
		 P = P0.copy();
	}
	
	public void update(long  tNew_nS) {
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
		RealMatrix F = getJacobian(dt_S);
		P = F.multiply(P).multiply(F.transpose()).add(Q.scalarMultiply(dt_S));
	}
	
	public void fuseGps(long timestamp, double lon_DEG, double lat_DEG, double alt_M, double acc_M) {
		// TODO implement this!
	}
	
	private RealMatrix getJacobian(double dt_S) {
		RealMatrix F = MatrixUtils.createRealIdentityMatrix(34);
		
		return F;
	}
	
	private static RealMatrix getProcessError() {
		double[] diag = new double[34];
		Arrays.fill(diag, 0.0);
		diag[6] = diag[7] = diag[8] = ACCELERATION_NOISE_M2pS4pS;
		diag[12] = diag[13] = diag[14] = ROTATION_RATE_NOISE_RAD2pS2pS;
		return MatrixUtils.createRealDiagonalMatrix(diag);
	}
	
	// *** rotation jacobian ***
	private final void rotationJacobian(Array2DRowRealMatrix J, Vector3D x, Quaternion q) {	
		
	}
}
