package com.graphhopper.android.sensor.observer;


/**
 * An gyroscope sensor observer interface. Classes that need to observe the
 * gyroscope sensor for updates should do so with this interface.
 */
public interface GyroscopeSensorObserver
{
	/**
	 * Notify observers when new gyroscope measurements are available.
	 * @param gyroscope the rotation values (x, y, z)
	 * @param timeStamp the time of the sensor update.
	 */
	public void onGyroscopeSensorChanged(float[] gyroscope, long timeStamp);
}
