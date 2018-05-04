package deadreckoning.activity;

import android.annotation.TargetApi;
import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;


import deadreckoning.R;
import deadreckoning.dialog.StepCalibrationDialogFragment;
import deadreckoning.extra.ExtraFunctions;
import deadreckoning.stepcounting.DynamicStepCounter;


public class StepCalibrationActivity extends Activity implements SensorEventListener {

    private TextView textAndroidSteps;
    private TextView textCalibrationDistance;
    private TextView textInstantAcc;

    private Button buttonStartCalibration;
    private Button buttonStopCalibration;
    private Button buttonSetStrideLength;

    private Sensor linearAcceleration;
    private Sensor androidStepCounter;
    private SensorManager sensorManager;

    private DynamicStepCounter[] dynamicStepCounters;

    private static int stepCount;
    private static int preferredStepCounterIndex;

    private boolean wasRunning;

    @TargetApi(Build.VERSION_CODES.KITKAT)
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_step_calibration);

        stepCount = 0;

        dynamicStepCounters = new DynamicStepCounter[20];

        double sensitivity = 0.5;
        for (int i = 0; i < dynamicStepCounters.length; i++) {
            dynamicStepCounters[i] = new DynamicStepCounter(sensitivity);
            sensitivity += 0.05;
        }


        textAndroidSteps = findViewById(R.id.textCalibrateSteps);
        textCalibrationDistance = findViewById(R.id.textCalibrationDistance);
        textInstantAcc = findViewById(R.id.textCalibrateInstantAcc);

        buttonStartCalibration = findViewById(R.id.buttonStartCalibration);
        buttonStopCalibration = findViewById(R.id.buttonStopCalibration);
        buttonSetStrideLength = findViewById(R.id.buttonSetStrideLength);


        sensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
        linearAcceleration = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
        androidStepCounter = sensorManager.getDefaultSensor(Sensor.TYPE_STEP_DETECTOR);


        buttonStartCalibration.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                sensorManager.registerListener(StepCalibrationActivity.this, linearAcceleration, SensorManager.SENSOR_DELAY_FASTEST);
                sensorManager.registerListener(StepCalibrationActivity.this, androidStepCounter, SensorManager.SENSOR_DELAY_FASTEST);

                buttonStartCalibration.setEnabled(false);
                buttonSetStrideLength.setEnabled(false);
                buttonStopCalibration.setEnabled(true);

                wasRunning = true;
            }
        });


        buttonStopCalibration.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                sensorManager.unregisterListener(StepCalibrationActivity.this);

                String[] stepCounts = new String[dynamicStepCounters.length];
                for (int i = 0; i < stepCounts.length; i++)
                    stepCounts[i] = String.format("%.2f : %d", dynamicStepCounters[i].getSensitivity(), dynamicStepCounters[i].getStepCount());


                // 对话框
                StepCalibrationDialogFragment stepCalibrationDialogFragment = new StepCalibrationDialogFragment(stepCounts, new StepCalibrationHandler(getApplicationContext()));
                stepCalibrationDialogFragment.show(getFragmentManager(), "step_counters");

                buttonStartCalibration.setEnabled(true);
                buttonSetStrideLength.setEnabled(true);
                buttonStopCalibration.setEnabled(false);

                wasRunning = false;
            }
        });

        // TODO：步长strideLength
        buttonSetStrideLength.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                double strideLength;
                if (stepCount != 0) {
                    strideLength = (double) Integer.parseInt(textCalibrationDistance.getText().toString()) / stepCount;
                } else {
                    Toast.makeText(getApplication(), "Take a few steps first!", Toast.LENGTH_SHORT).show();
                    return;
                }

                Toast.makeText(getApplicationContext(), "Stride length set: " + strideLength + " ft/step", Toast.LENGTH_SHORT).show();

                //将stride_length和preferred_step_counter信息返回给调用活动
                Intent myIntent = getIntent();
                myIntent.putExtra("stride_length", strideLength);
                myIntent.putExtra("preferred_step_counter", dynamicStepCounters[preferredStepCounterIndex].getSensitivity());
                setResult(RESULT_OK, myIntent);
                finish();

            }
        });

    }

    @Override
    protected void onStop() {
        super.onStop();
        sensorManager.unregisterListener(this);
    }

    @Override
    protected void onResume() {
        super.onResume();

        if (wasRunning) {
            sensorManager.registerListener(StepCalibrationActivity.this, linearAcceleration, SensorManager.SENSOR_DELAY_FASTEST);
            sensorManager.registerListener(StepCalibrationActivity.this, androidStepCounter, SensorManager.SENSOR_DELAY_FASTEST);

            buttonStartCalibration.setEnabled(false);
            buttonSetStrideLength.setEnabled(false);
            buttonStopCalibration.setEnabled(true);
        } else {
            buttonStartCalibration.setEnabled(true);
            buttonSetStrideLength.setEnabled(true);
            buttonStopCalibration.setEnabled(false);
        }

    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {}

    @Override
    public void onSensorChanged(SensorEvent event) {

        if (event.sensor.getType() == Sensor.TYPE_STEP_DETECTOR) {
            if (event.values[0] == 1.0) {
                stepCount++;
                textAndroidSteps.setText(String.valueOf(stepCount));
            }
        } else if (event.sensor.getType() == Sensor.TYPE_LINEAR_ACCELERATION) {

            //TODO：矫正线性加速度
            float norm = ExtraFunctions.calcNorm(
                    event.values[0] +
                    event.values[1] +
                    event.values[2]
            );

            textInstantAcc.setText(String.valueOf(event.values[2]).substring(0, 5));

            for(DynamicStepCounter dynamicStepCounter : dynamicStepCounters)
                dynamicStepCounter.findStep(norm);
        }

    }

    public static class StepCalibrationHandler extends Handler {

        Context context;

        public StepCalibrationHandler(Context context) {
            this.context = context;
        }

        @Override
        public void handleMessage(Message msg) {
            super.handleMessage(msg);

            //preferred_step_counter的索引存储在空白消息的“what”字段中
            preferredStepCounterIndex = msg.what;

        }
    }

}
