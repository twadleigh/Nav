package com.github.twadleigh.nav;

import org.apache.commons.math3.complex.Quaternion;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

public class Nav {
	
	static {
		System.loadLibrary("NavJni");
	}

	public static native int theAnswer();
	
	// *** state ***
	
	// origin of ENU frame
	private Vector3D origin_LLA;
	
	// time (nanoseconds since startup)
	private long t_nS;

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
	
	// state estimate
	private RealVector x = new ArrayRealVector(SIZE);
	private RealMatrix P = new Array2DRowRealMatrix(34,34);
	
	// parameters
	private static RealVector x0; // initial state
	private static RealMatrix P0; // initial uncertainty
	private static RealMatrix Q; // process error
	private static RealMatrix Rgps; // GPS noise
	private static RealMatrix Racc; // accelerometer noise
	private static RealMatrix Rgyr; // gyroscope noise
	private static RealMatrix Rmag; // magnetometer noise
	
	public Nav() {
		 x = x0.copy();
		 P = P0.copy();
	}
	/*
	public void update(long  tNew_nS) {
		if(tNew_nS == t_nS) return;
		
		double dt_S = 1.0e-9*((double)(tNew_nS-t_nS));
		
		// update time
		t_nS = tNew_nS;
		
		// update position
		double hdt2 = 0.5*dt_S*dt_S;
		x.setEntry(POSITION+0, dt_S*x.getEntry(VELOCITY+0) + hdt2*x.getEntry(ACCELERATION+0));
		x.setEntry(POSITION+1, dt_S*x.getEntry(VELOCITY+1) + hdt2*x.getEntry(ACCELERATION+1));
		x.setEntry(POSITION+2, dt_S*x.getEntry(VELOCITY+2) + hdt2*x.getEntry(ACCELERATION+2));
		
		// update velocity
		x.setEntry(VELOCITY+0, dt_S*x.getEntry(ACCELERATION+0));
		x.setEntry(VELOCITY+1, dt_S*x.getEntry(ACCELERATION+1));
		x.setEntry(VELOCITY+2, dt_S*x.getEntry(ACCELERATION+2));
		
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
	*/
	// *** rotation jacobian ***
	private final void rotationJacobian(Array2DRowRealMatrix J, Vector3D x, Quaternion q) {	
		
	}
}
