package com.graphhopper.android.sensor.observer;


/**
 * An acceleration sensor observer interface. Classes that need to observe the
 * acceleration sensor for updates should do so with this interface.
 */
public interface AccelerationSensorObserver
{
	/**
	 * Notify observers when new acceleration measurements are available.
	 * @param acceleration the acceleration values (x, y, z)
	 * @param timeStamp the time of the sensor update.
	 */
	public void onAccelerationSensorChanged(float[] acceleration,
                                            long timeStamp);
}
