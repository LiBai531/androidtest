package com.graphhopper.android.sensor.observer;


public interface LinearAccelerationSensorObserver
{

	public void onLinearAccelerationSensorChanged(float[] linearAcceleration, long timeStamp);
}
