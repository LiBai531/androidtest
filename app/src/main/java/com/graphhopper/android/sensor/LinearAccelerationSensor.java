package com.graphhopper.android.sensor;

import android.content.Context;
import android.hardware.SensorManager;

import com.graphhopper.android.filters.MeanFilter;
import com.graphhopper.android.sensor.observer.AccelerationSensorObserver;
import com.graphhopper.android.sensor.observer.GravitySensorObserver;
import com.graphhopper.android.sensor.observer.GyroscopeSensorObserver;
import com.graphhopper.android.sensor.observer.LinearAccelerationSensorObserver;
import com.graphhopper.android.sensor.observer.MagneticSensorObserver;

import java.util.ArrayList;

public class LinearAccelerationSensor implements GyroscopeSensorObserver,
		AccelerationSensorObserver, MagneticSensorObserver, GravitySensorObserver
{
	public static final float EPSILON = 0.000000001f;

	private static final float NS2S = 1.0f / 1000000000.0f;
	private static final int MEAN_FILTER_WINDOW = 10;
	private static final int MIN_SAMPLE_COUNT = 30;

	// Keep track of observers.
	private ArrayList<LinearAccelerationSensorObserver> observersAcceleration;

	private boolean hasInitialOrientation = false;
	private boolean stateInitialized = false;

	private Context context;

	private long timestampOld = 0;

	// Calibrated maths.
	private float[] currentRotationMatrix;
	private float[] deltaRotationMatrix;
	private float[] deltaRotationVector;
	private float[] gyroscopeOrientation;

	// 加速度信号的重力分量。
	private float[] components = new float[3];

	private float[] linearAcceleration = new float[]
	{ 0, 0, 0 };

	// 初始accelerometer数据
	private float[] acceleration = new float[]
	{ 0, 0, 0 };

	// 初始gravity数据
	private float[] gravity = new float[]
			{ 0, 0, 0 };

	// 初始magnetic数据
	private float[] magnetic = new float[]
	{ 0, 0, 0 };

	private int gravitySampleCount = 0;
	private int magneticSampleCount = 0;

	private MeanFilter mfAcceleration;
	private MeanFilter mfMagnetic;
	private MeanFilter mfGravity;
	private MeanFilter mfLinearAcceleration;

	// 旋转矩阵R将矢量从设备坐标系转换为定义为直接标准正交基的世界坐标系。R是设备与世界坐标系对齐时的单位矩阵，即设备的X轴指向东方时，轴指向北极并且设备面向天空。
	// getOrientation使用的参考坐标系与为旋转矩阵R和getRotationMatrix定义的世界坐标系不同。
	private float[] initialRotationMatrix;

	private GravitySensor gravitySensor;
	private GyroscopeSensor gyroscopeSensor;
	private MagneticSensor magneticSensor;

	public LinearAccelerationSensor(Context context)
	{
		super();

		this.context = context;
		observersAcceleration = new ArrayList<LinearAccelerationSensorObserver>();

		initFilters();
		initSensors();
		reset();
		restart();
	}

	public void onStart()
	{
		restart();
	}

	public void onPause()
	{
		reset();
	}

	@Override
	public void onAccelerationSensorChanged(float[] acceleration, long timeStamp)
	{
		System.arraycopy(acceleration, 0, this.acceleration, 0, acceleration.length);
	}
	
	@Override
	public void onGravitySensorChanged(float[] gravity, long timeStamp)
	{
		System.arraycopy(gravity, 0, this.gravity, 0, gravity.length);

		this.gravity = mfGravity.filterFloat(this.gravity);

		gravitySampleCount++;

		// 只有在加速度传感器和磁性传感器有足够的时间用平均滤波器平滑后才能确定初始方向。只有在方向尚未确定的情况进行。
		if (gravitySampleCount > MIN_SAMPLE_COUNT
				&& magneticSampleCount > MIN_SAMPLE_COUNT
				&& !hasInitialOrientation)
		{
			calculateOrientation();
		}
		
	}

	//TODO:就是这里实现liner
	@Override
	public void onGyroscopeSensorChanged(float[] gyroscope, long timestamp)
	{
		// 在获取第一个加速度计/磁力计方向之前不启动
		if (!hasInitialOrientation)
		{
			return;
		}

		// 基于旋转矩阵初始化gyroscope
		if (!stateInitialized)
		{
			currentRotationMatrix = matrixMultiplication(currentRotationMatrix, initialRotationMatrix);

			stateInitialized = true;
		}

		//从陀螺仪采样数据计算出来后，该时间步的增量旋转量将乘以当前旋转量。
		if (timestampOld != 0 && stateInitialized)
		{
			final float dT = (timestamp - timestampOld) * NS2S;

			// Axis of the rotation sample, not normalized yet.
			float axisX = gyroscope[0];
			float axisY = gyroscope[1];
			float axisZ = gyroscope[2];

			// 计算采样的角速度
			float omegaMagnitude = (float) Math.sqrt(axisX * axisX + axisY * axisY + axisZ * axisZ);

			// 如果旋转矢量足够大可以获取axis，则将其标准化
			if (omegaMagnitude > EPSILON)
			{
				axisX /= omegaMagnitude;
				axisY /= omegaMagnitude;
				axisZ /= omegaMagnitude;
			}


			float thetaOverTwo = omegaMagnitude * dT / 2.0f;

			float sinThetaOverTwo = (float) Math.sin(thetaOverTwo);
			float cosThetaOverTwo = (float) Math.cos(thetaOverTwo);

			//四元数
			deltaRotationVector[0] = sinThetaOverTwo * axisX;
			deltaRotationVector[1] = sinThetaOverTwo * axisY;
			deltaRotationVector[2] = sinThetaOverTwo * axisZ;
			deltaRotationVector[3] = cosThetaOverTwo;

			SensorManager.getRotationMatrixFromVector(deltaRotationMatrix, deltaRotationVector);

			currentRotationMatrix = matrixMultiplication(currentRotationMatrix, deltaRotationMatrix);

			SensorManager.getOrientation(currentRotationMatrix, gyroscopeOrientation);

			// values[0]: azimuth, rotation around the Z axis.
			// values[1]: pitch, rotation around the X axis.
			// values[2]: roll, rotation around the Y axis.

			// Find the gravity component of the X-axis
			// = g*-cos(pitch)*sin(roll);
			components[0] = (float) (SensorManager.GRAVITY_EARTH * -Math.cos(gyroscopeOrientation[1]) * Math.sin(gyroscopeOrientation[2]));

			// Find the gravity component of the Y-axis
			// = g*-sin(pitch);
			components[1] = (float) (SensorManager.GRAVITY_EARTH * -Math.sin(gyroscopeOrientation[1]));

			// Find the gravity component of the Z-axis
			// = g*cos(pitch)*cos(roll);
			components[2] = (float) (SensorManager.GRAVITY_EARTH * Math.cos(gyroscopeOrientation[1]) * Math.cos(gyroscopeOrientation[2]));

			linearAcceleration[0] = (this.acceleration[0] - components[0]);
			linearAcceleration[1] = (this.acceleration[1] - components[1]);
			linearAcceleration[2] = (this.acceleration[2] - components[2]);

			linearAcceleration = mfLinearAcceleration.filterFloat(linearAcceleration);
		}

		timestampOld = timestamp;

		notifyLinearAccelerationObserver();
	}

	@Override
	public void onMagneticSensorChanged(float[] magnetic, long timeStamp)
	{
		System.arraycopy(magnetic, 0, this.magnetic, 0, magnetic.length);

		this.magnetic = mfMagnetic.filterFloat(this.magnetic);

		magneticSampleCount++;
	}


	private void notifyLinearAccelerationObserver()
	{
		for (LinearAccelerationSensorObserver a : observersAcceleration)
		{
			a.onLinearAccelerationSensorChanged(this.linearAcceleration, this.timestampOld);
		}
	}


	public void registerAccelerationObserver(LinearAccelerationSensorObserver observer)
	{

		int i = observersAcceleration.indexOf(observer);
		if (i == -1)
		{
			observersAcceleration.add(observer);
		}

	}


	public void removeAccelerationObserver(LinearAccelerationSensorObserver observer)
	{
		int i = observersAcceleration.indexOf(observer);
		if (i >= 0)
		{
			observersAcceleration.remove(i);
		}
	}


	private void calculateOrientation()
	{
		hasInitialOrientation = SensorManager.getRotationMatrix(initialRotationMatrix, null, gravity, magnetic);

		if (hasInitialOrientation)
		{
			gravitySensor.removeGravityObserver(this);
			magneticSensor.removeMagneticObserver(this);
		}
	}


	private void initFilters()
	{
		mfAcceleration = new MeanFilter();
		mfAcceleration.setWindowSize(MEAN_FILTER_WINDOW);
		
		mfGravity = new MeanFilter();
		mfGravity.setWindowSize(MEAN_FILTER_WINDOW);

		mfLinearAcceleration = new MeanFilter();
		mfLinearAcceleration.setWindowSize(MEAN_FILTER_WINDOW);

		mfMagnetic = new MeanFilter();
		mfMagnetic.setWindowSize(MEAN_FILTER_WINDOW);
	}


	private void initMaths()
	{
		acceleration = new float[3];
		magnetic = new float[3];

		initialRotationMatrix = new float[9];

		deltaRotationVector = new float[4];
		deltaRotationMatrix = new float[9];
		currentRotationMatrix = new float[9];
		gyroscopeOrientation = new float[3];


		currentRotationMatrix[0] = 1.0f;
		currentRotationMatrix[4] = 1.0f;
		currentRotationMatrix[8] = 1.0f;
	}


	private void initSensors()
	{
		gravitySensor = new GravitySensor(context);
		magneticSensor = new MagneticSensor(context);
		gyroscopeSensor = new GyroscopeSensor(context);
	}

	/**
	 * 
	 * @param a
	 * @param b
	 * @return a*b
	 */
	private float[] matrixMultiplication(float[] a, float[] b)
	{
		float[] result = new float[9];

		result[0] = a[0] * b[0] + a[1] * b[3] + a[2] * b[6];
		result[1] = a[0] * b[1] + a[1] * b[4] + a[2] * b[7];
		result[2] = a[0] * b[2] + a[1] * b[5] + a[2] * b[8];

		result[3] = a[3] * b[0] + a[4] * b[3] + a[5] * b[6];
		result[4] = a[3] * b[1] + a[4] * b[4] + a[5] * b[7];
		result[5] = a[3] * b[2] + a[4] * b[5] + a[5] * b[8];

		result[6] = a[6] * b[0] + a[7] * b[3] + a[8] * b[6];
		result[7] = a[6] * b[1] + a[7] * b[4] + a[8] * b[7];
		result[8] = a[6] * b[2] + a[7] * b[5] + a[8] * b[8];

		return result;
	}


	private void restart()
	{
		gravitySensor.registerGravityObserver(this);
		magneticSensor.registerMagneticObserver(this);
		gyroscopeSensor.registerGyroscopeObserver(this);
	}


	private void reset()
	{
		gravitySensor.removeGravityObserver(this);
		magneticSensor.removeMagneticObserver(this);
		gyroscopeSensor.removeGyroscopeObserver(this);

		initMaths();

		gravitySampleCount = 0;
		magneticSampleCount = 0;

		hasInitialOrientation = false;
		stateInitialized = false;
	}
}
