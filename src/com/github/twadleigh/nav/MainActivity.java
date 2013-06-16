package com.github.twadleigh.nav;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Bundle;
import android.app.Activity;
import android.content.Context;
import android.view.Menu;

public class MainActivity extends Activity implements LocationListener, SensorEventListener {
	
	private LocationManager mLocationManager;
	private SensorManager mSensorManager;
	private Sensor mAccelerometer;
	private Sensor mGyroscope;
	private Sensor mMagneticField;
	
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        
        // set up location services
        mLocationManager = (LocationManager) this.getSystemService(Context.LOCATION_SERVICE);
        
        // set up sensor services
        mSensorManager = (SensorManager) this.getSystemService(Context.SENSOR_SERVICE);
        mAccelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        mGyroscope = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        mMagneticField = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);

        // initialize Nav
        Location loc = mLocationManager.getLastKnownLocation(LocationManager.GPS_PROVIDER);
        Nav.initialize(
        		loc.getElapsedRealtimeNanos(),
        		loc.getLongitude(),
        		loc.getLatitude(),
        		loc.getAltitude(),
        		loc.getAccuracy());
    }
    
    @Override
    protected void onResume() {
    	super.onResume();
        mLocationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 0, 0, this);
        mSensorManager.registerListener(this, mAccelerometer, SensorManager.SENSOR_DELAY_FASTEST);
    	mSensorManager.registerListener(this, mGyroscope, SensorManager.SENSOR_DELAY_FASTEST);
    	mSensorManager.registerListener(this, mMagneticField, SensorManager.SENSOR_DELAY_FASTEST);
    }
    
    @Override
    protected void onPause() {
    	mLocationManager.removeUpdates(this);
    	mSensorManager.unregisterListener(this);
    	super.onPause();
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.main, menu);
        return true;
    }
    
	@Override
	public void onLocationChanged(Location loc) {
		Nav.updateGps(
        		loc.getElapsedRealtimeNanos(),
        		loc.getLongitude(),
        		loc.getLatitude(),
        		loc.getAltitude(),
        		loc.getAccuracy());
	}

	@Override
	public void onSensorChanged(SensorEvent evt) {
		long t = evt.timestamp;
		float[] v = evt.values;
		if(evt.sensor == mAccelerometer) {
			Nav.updateAccelerometer(t,v);
		} else if(evt.sensor == mGyroscope) {
			Nav.updateGyroscope(t,v);
		} else if(evt.sensor == mMagneticField) {
			Nav.updateMagnetometer(t,v);
		}
	}

	@Override
	public void onProviderDisabled(String arg0) {
		// do nothing
	}

	@Override
	public void onProviderEnabled(String arg0) {
		// do nothing
	}

	@Override
	public void onStatusChanged(String arg0, int arg1, Bundle arg2) {
		// do nothing
	}

	@Override
	public void onAccuracyChanged(Sensor arg0, int arg1) {
		// do nothing
	}
}
