package com.github.twadleigh.nav;

public class Nav {
	
	static {
		System.loadLibrary("NavJni");
	}

	// initialization
	public static native void initialize(long t0, double lon, double lat, double alt, float err);
	
	// measurement updates
	public static native void updateGps(long t, double lon, double lat, double alt, float err);
	public static native void updateAccelerometer(long t, float[] a);
	public static native void updateGyroscope(long t, float[] o);
	public static native void updateMagnetometer(long t, float[] f);
	
	// state query
	public static native void getState(long t, double[] s);
}
