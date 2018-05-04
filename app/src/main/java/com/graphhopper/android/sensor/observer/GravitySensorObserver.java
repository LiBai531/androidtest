package com.graphhopper.android.sensor.observer;


/**
 * An gravity sensor observer interface. Classes that need to observe the
 * acceleration sensor for updates should do so with this interface.
 */
public interface GravitySensorObserver
{
	/**
	 * Notify observers when new gravity measurements are available.
	 * @param gravity the gravity values (x, y, z)
	 * @param timeStamp the time of the sensor update.
	 */
	public void onGravitySensorChanged(float[] gravity,
                                       long timeStamp);
}
