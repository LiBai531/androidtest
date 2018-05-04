package com.graphhopper.android.sensor.observer;


/**
 * An magnetic sensor observer interface. Classes that need to observe the
 * magnetic sensor for updates should do so with this interface.
 *
 */
public interface MagneticSensorObserver
{
	/**
	 * Notify observers when new magnetic measurements are available.
	 * 
	 * @param magnetic
	 *            the magnetic measurements (x, y, z).
	 * @param timeStamp
	 *            the time stamp of the measurement.
	 */
	public void onMagneticSensorChanged(float[] magnetic, long timeStamp);
}
