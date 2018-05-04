package com.graphhopper.android.sensor;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;


import com.graphhopper.android.sensor.observer.MagneticSensorObserver;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import java.util.ArrayList;


public class MagneticSensor implements SensorEventListener
{


	private ArrayList<MagneticSensorObserver> observersMagnetic;

	// Keep track of the application mode. Vehicle Mode occurs when the device
	// is in the Landscape orientation and the sensors are rotated to face the
	// -Z-Axis (along the axis of the camera).
	private boolean vehicleMode = false;

	private Context context;

	private float[] magnetic = new float[3];

	private long timeStamp = 0;

	// Quaternion data structures to rotate a matrix from the absolute Android
	// orientation to the orientation that the device is actually in. This is
	// needed because the the device sensors orientation is fixed in hardware.
	// Also remember the many algorithms require a NED orientation which is not
	// the same as the absolute Android orientation. Do not confuse this
	// rotation with a rotation into absolute earth frame!
	private Rotation yQuaternion;
	private Rotation xQuaternion;
	private Rotation rotationQuaternion;

	// We need the SensorManager to register for Sensor Events.
	private SensorManager sensorManager;

	// The vectors that will be rotated when the application is in Vehicle Mode.
	private Vector3D vIn;
	private Vector3D vOut;

	/**
	 * Initialize the state.
	 * 
	 * @param context
	 *            the Activities context.
	 */
	public MagneticSensor(Context context)
	{
		super();

		this.context = context;

		initQuaternionRotations();

		observersMagnetic = new ArrayList<MagneticSensorObserver>();

		sensorManager = (SensorManager) this.context.getSystemService(Context.SENSOR_SERVICE);
	}

	/**
	 * Register for Sensor.TYPE_MAGNETIC measurements.
	 *
	 * @param observer
	 *            The observer to be registered.
	 */
	public void registerMagneticObserver(LinearAccelerationSensor observer)
	{
		if (observersMagnetic.size() == 0)
		{
			sensorManager.registerListener(this, sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD), SensorManager.SENSOR_DELAY_FASTEST);
		}

		int i = observersMagnetic.indexOf(observer);
		if (i == -1)
		{
			observersMagnetic.add(observer);
		}
	}

	/**
	 * Remove Sensor.TYPE_MAGNETIC measurements.
	 *
     * @param observer
     *            The observer to be removed.
     */
	public void removeMagneticObserver(LinearAccelerationSensor observer)
	{
		int i = observersMagnetic.indexOf(observer);
		if (i >= 0)
		{
			observersMagnetic.remove(i);
		}

		if (observersMagnetic.size() == 0)
		{
			sensorManager.unregisterListener(this);
		}
	}

	@Override
	public void onAccuracyChanged(Sensor sensor, int accuracy)
	{

	}

	@Override
	public void onSensorChanged(SensorEvent event)
	{
		if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD)
		{
			System.arraycopy(event.values, 0, magnetic, 0, event.values.length);

			timeStamp = event.timestamp;

			if (vehicleMode)
			{
				this.magnetic = quaternionToDeviceVehicleMode(this.magnetic);
			}

			notifyMagneticObserver();
		}
	}

	/**
	 * Vehicle mode occurs when the device is put into the landscape
	 * orientation. On Android phones, the positive Y-Axis of the sensors faces
	 * towards the top of the device. In vehicle mode, we want the sensors to
	 * face the negative Z-Axis so it is aligned with the camera of the device.
	 * 
	 * @param vehicleMode
	 *            true if in vehicle mode.
	 */
	public void setVehicleMode(boolean vehicleMode)
	{
		this.vehicleMode = vehicleMode;
	}

	/**
	 * To avoid anomalies at the poles with Euler angles and Gimbal lock,
	 * quaternions are used instead.
	 */
	private void initQuaternionRotations()
	{
		// Rotate by 90 degrees or pi/2 radians.
		double rotation = Math.PI / 2;

		// Create the rotation around the x-axis
		Vector3D xV = new Vector3D(1, 0, 0);
		xQuaternion = new Rotation(xV, rotation);

		// Create the rotation around the y-axis
		Vector3D yV = new Vector3D(0, 1, 0);
		yQuaternion = new Rotation(yV, -rotation);

		// Create the composite rotation.
		rotationQuaternion = yQuaternion.applyTo(xQuaternion);
	}
	
	/**
	 * Notify observers with new measurements.
	 */
	private void notifyMagneticObserver()
	{
		for (MagneticSensorObserver a : observersMagnetic)
		{
			a.onMagneticSensorChanged(this.magnetic, this.timeStamp);
		}
	}

	private float[] quaternionToDeviceVehicleMode(float[] matrix)
	{
		vIn = new Vector3D(matrix[0], matrix[1], matrix[2]);
		vOut = rotationQuaternion.applyTo(vIn);

		float[] rotation =
		{ (float) vOut.getX(), (float) vOut.getY(), (float) vOut.getZ() };

		return rotation;
	}
}
