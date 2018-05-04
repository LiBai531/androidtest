package com.graphhopper.android.mfsensor;

import android.Manifest;
import android.annotation.SuppressLint;
import android.app.Dialog;
import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.os.Environment;
import android.os.Handler;
import android.preference.PreferenceManager;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.support.v7.app.AppCompatActivity;
import android.support.v7.widget.Toolbar;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.view.Window;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import com.graphhopper.GraphHopper;
import com.graphhopper.reader.osm.GraphHopperOSM;
import com.graphhopper.routing.AlgorithmOptions;
import com.graphhopper.routing.util.CarFlagEncoder;
import com.graphhopper.routing.util.EncodingManager;
import com.graphhopper.routing.weighting.FastestWeighting;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.util.GPXEntry;
import com.graphhopper.util.Parameters;
import com.kircherelectronics.fsensor.filter.averaging.AveragingFilter;
import com.kircherelectronics.fsensor.filter.averaging.LowPassFilter;
import com.kircherelectronics.fsensor.filter.averaging.MeanFilter;
import com.kircherelectronics.fsensor.filter.averaging.MedianFilter;
import com.kircherelectronics.fsensor.filter.gyroscope.OrientationGyroscope;
import com.kircherelectronics.fsensor.filter.gyroscope.fusion.complimentary.OrientationFusedComplimentary;
import com.kircherelectronics.fsensor.filter.gyroscope.fusion.kalman.OrientationFusedKalman;
import com.kircherelectronics.fsensor.linearacceleration.LinearAcceleration;
import com.kircherelectronics.fsensor.linearacceleration.LinearAccelerationAveraging;
import com.kircherelectronics.fsensor.linearacceleration.LinearAccelerationFusion;
import com.kircherelectronics.fsensor.util.rotation.RotationUtil;

import org.apache.commons.math3.complex.Quaternion;
import org.ejml.data.SimpleMatrix;

import java.io.FileWriter;
import java.sql.Timestamp;
import java.util.ArrayList;
import java.util.List;

import datalogger.DataLoggerManager;
import deadreckoning.R;
import matching.EdgeMatch;
import matching.GPXFile;
import matching.MapMatching;
import matching.MatchResult;


public class GyroscopeActivity extends AppCompatActivity implements SensorEventListener {



    private boolean logData = false;
    private DataLoggerManager dataLogger;
    int i = 0;

    private MeanFilter averagingFilter;
    private LinearAcceleration linearAccelerationFilter;

    final private int numbers = 30;
    private boolean zeroV1 = false;
    private boolean zeroV2 = false;
    private boolean zeroV3 = false;
    private ArrayList<GPXEntry> mGPXEntries = new ArrayList<>();


    final double alpha = 0.8;

    long etime = 0; //最新传感器数据的时间
    long ptAcc = 0, ptGyro = 0, ptMag = 0;    //上一个采样时间
    private final float N2S = 1000000000f;

    long Rearth = 6371004; //m

//    private SimpleMatrix sums = new SimpleMatrix(1, 3);
    private SimpleMatrix tranamx = new SimpleMatrix(1, 3);

    FileWriter gpxLogWriter;

    private SimpleMatrix ntoe = new SimpleMatrix(3, 3);
    private SimpleMatrix mA = new SimpleMatrix(1,3);
    private SimpleMatrix mV = new SimpleMatrix(1,3);
    private SimpleMatrix mX = new SimpleMatrix(1,3);
    private SimpleMatrix delX = new SimpleMatrix(1,3);
    private SimpleMatrix mA_pre = new SimpleMatrix(1,3);
    private SimpleMatrix mV_pre = new SimpleMatrix(1,3);

    private final static int WRITE_EXTERNAL_STORAGE_REQUEST = 1000;

    private LowPassFilter lpfAccelerationSmoothing;
    private MeanFilter meanFilterAccelerationSmoothing;
    private MedianFilter medianFilterAccelerationSmoothing;
    private OrientationFusedComplimentary orientationFusionComplimentary;

    private LinearAcceleration linearAccelerationFilterComplimentary;
    private LinearAcceleration linearAccelerationFilterKalman;
    private LinearAcceleration linearAccelerationFilterLpf;
    private AveragingFilter lpfGravity;

    private Trans mTrans = new Trans();
    private SimpleMatrix tntob = new SimpleMatrix(3, 3);


    private boolean hasAcceleration = false;
    private boolean hasMagnetic = false;

    private boolean meanFilterEnabled;
    private boolean kalmanFilterEnabled;
    private boolean complimentaryFilterEnabled;

    private float[] fusedOrientation = new float[3];
    private float[] acceleration = new float[3];
    private float[] magnetic = new float[3];
    private float[] rotation = new float[3];
    private float[] transacc = new float[3];
    private float[] degret = new float[3];

    public SimpleMatrix result = new SimpleMatrix(1, 3);

    private Mode mode = Mode.COMPLIMENTARY_FILTER;

    protected Handler handler;

    protected Runnable runable;

    private TextView tvXAxis;
    private TextView tvYAxis;
    private TextView tvZAxis;

    private TextView LineX;
    private TextView LineY;
    private TextView AX;
    private TextView AY;
    private TextView VX;
    private TextView VY;

    private OrientationGyroscope orientationGyroscope;
    private OrientationFusedComplimentary orientationComplimentaryFusion;
    private OrientationFusedKalman orientationKalmanFusion;

    private MeanFilter meanFilter;

    private SensorManager sensorManager;

    double initLongitude;
    double initLatitude;
    double initEle;

    double longitude;
    double latitude;
    double ele;

    private String TAG = "lxy";
    private GyroscopeBias gyroUBias;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        setContentView(R.layout.activity_gyroscope);
        meanFilter = new MeanFilter();
        dataLogger = new DataLoggerManager(this);
        sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        //database = SQLiteDatabase.openOrCreateDatabase(DBManager.DB_PATH + "/" + DBManager.DB_NAME, null);



        Intent intent = getIntent();
        initLongitude = intent.getDoubleExtra("lon", 0.00);
        initLatitude = intent.getDoubleExtra("lat", 0.00);
        initEle = intent.getDoubleExtra("ele", 0.00);

        ntoe = mTrans.eton(initLongitude, initLatitude);
        //Log.i("lxy", "lon: " + initLongitude);
        ntoe.print();
        //TODO:提前初始化？
        gyroUBias = new GyroscopeBias(100);


        initUI();
        init();
        initStartButton();
    }



    private void initStartButton() {
        final Button button = findViewById(R.id.button_start);
        Button gps_start = findViewById(R.id.gps_start);
        Button gps_stop = findViewById(R.id.gps_stop);
        Button clr_v = findViewById(R.id.clr_v);
        final Button clr_all = findViewById(R.id.button_stop);
        final Button match = findViewById(R.id.match);


        //TODO:如果不记录？直接做实体的匹配？
        match.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                Intent intent = new Intent(GyroscopeActivity.this,Matcher.class);
                startActivityForResult(intent,1);

            }
        });

        clr_v.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                clr_v();
            }
        });

        clr_all.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                clr_all();
            }
        });

        button.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                if (!logData) {
                    button.setText("Stop Log");
                    startDataLog();
                } else {
                    button.setText("Start Log");
                    stopDataLog();
                }
            }
        });
        gps_start.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                startLogging();
            }
        });
        gps_stop.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                stopLogging();
            }
        });
    }

    private void clr_v() {
        mV.set(0,0.0);
        mV.set(1,0.0);
        mV.set(2,0.0);
    }

    private void clr_all() {
        mV.set(0,0.0);
        mV.set(1,0.0);
        mV.set(2,0.0);
        mX.set(0,0.0);
        mX.set(1,0.0);
        mX.set(2,0.0);
    }

    private void startDataLog() {
        logData = true;
        dataLogger.startDataLog();
    }

    private void stopDataLog() {
        logData = false;
        String path = dataLogger.stopDataLog();
        Toast.makeText(this, "File Written to: " + path, Toast.LENGTH_SHORT).show();
    }

    private void init() {





        averagingFilter = new MeanFilter(5);
        linearAccelerationFilter = new LinearAccelerationAveraging(averagingFilter);

        lpfAccelerationSmoothing = new LowPassFilter();
        meanFilterAccelerationSmoothing = new MeanFilter();
        medianFilterAccelerationSmoothing = new MedianFilter();
        lpfGravity = new LowPassFilter();
        orientationFusionComplimentary = new OrientationFusedComplimentary();
        OrientationFusedKalman orientationFusionKalman = new OrientationFusedKalman();


        linearAccelerationFilterComplimentary = new LinearAccelerationFusion(orientationFusionComplimentary);
        linearAccelerationFilterKalman = new LinearAccelerationFusion(orientationFusionKalman);
        linearAccelerationFilterLpf = new LinearAccelerationAveraging(lpfGravity);
        //sums.zero();


        //linearAccelerationFilter = new LinearAccelerationAveraging(averagingFilter);
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        getMenuInflater().inflate(R.menu.gyroscope, menu);
        return true;
    }


    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        switch (item.getItemId()) {

            // Reset
            case R.id.action_reset:
                if (orientationGyroscope != null) {
                    orientationGyroscope.reset();
                }
                return true;

            // config
            case R.id.action_config:
                Intent intent = new Intent();
                intent.setClass(this, ConfigActivity.class);
                startActivity(intent);
                return true;

            // help
            case R.id.action_help:
                showHelpDialog();
                return true;

            default:
                return super.onOptionsItemSelected(item);
        }
    }
    @Override
    protected void onActivityResult(int requestCode,int resultCode,Intent data){
        switch (requestCode){
            case 1:
                if (resultCode == RESULT_OK){
                    double[] lonlat = data.getDoubleArrayExtra("data_return");
                    longitude = lonlat[0];
                    latitude = lonlat[1];
                    clr_all();
                    Log.i(TAG, "data: OK    "+lonlat[0]+",  "+lonlat[1] );
                }
                break;
            default:
        }
    }


    @Override
    public void onResume() {
        super.onResume();

        requestPermissions();
        readPrefs();

        switch (mode) {
            case GYROSCOPE_ONLY:
                orientationGyroscope = new OrientationGyroscope();
                break;
            case COMPLIMENTARY_FILTER:
                orientationComplimentaryFusion = new OrientationFusedComplimentary();
                break;
            case KALMAN_FILTER:
                orientationKalmanFusion = new OrientationFusedKalman();
                break;
        }

        reset();

        if (orientationKalmanFusion != null) {
            orientationKalmanFusion.startFusion();
        }

        sensorManager.registerListener(this, sensorManager
                        .getDefaultSensor(Sensor.TYPE_ACCELEROMETER),
                SensorManager.SENSOR_DELAY_NORMAL);

        // Register for sensor updates.
        sensorManager.registerListener(this, sensorManager
                        .getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD),
                SensorManager.SENSOR_DELAY_NORMAL);

        // Register for sensor updates.
        sensorManager.registerListener(this,
                sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE),
                SensorManager.SENSOR_DELAY_NORMAL);

        handler.post(runable);
    }

    @Override
    public void onPause() {
        super.onPause();

        if (orientationKalmanFusion != null) {
            orientationKalmanFusion.stopFusion();
        }

    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        etime = event.timestamp;
        float dt = 0;

        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {

            System.arraycopy(event.values, 0, acceleration, 0, event.values.length);
            //TODO:一个问题就是线性加速度滞后太多

            //float[] linerAccelerationf = new float[3];

            averagingFilter .filter(acceleration);
            transacc = linearAccelerationFilter.filter(acceleration);
            //Log.i(TAG, "linerAccelerationf: "+linerAccelerationf[0]+",  "+linerAccelerationf[1]+",  "+linerAccelerationf[2]);

//            //TODO:这个加速度还有点问题，需要多矫正一下。
//            gravity[0] = alpha*gravity[0] + (1-alpha)*acceleration[0];
//            gravity[1] = alpha*gravity[1] + (1-alpha)*acceleration[1];
//            gravity[2] = alpha*gravity[2] + (1-alpha)*acceleration[2];
//
//            linerAcceleration[0] = acceleration[0] - gravity[0];
//            linerAcceleration[1] = acceleration[1] - gravity[1];
//            linerAcceleration[2] = acceleration[2] - gravity[2];
//
//
//            linerAccelerationf[0] = new Double(linerAcceleration[0]).floatValue();
//            linerAccelerationf[1] = new Double(linerAcceleration[1]).floatValue();
//            linerAccelerationf[2] = new Double(linerAcceleration[2]).floatValue();

            //transacc = meanFilterAccelerationSmoothing.filter(linerAccelerationf);
            if ((transacc[0] * transacc[0] + transacc[1] * transacc[1] + transacc[2] * transacc[2]) < 0.5) {
                transacc[0] = 0;
                transacc[1] = 0;
                transacc[2] = 0;
                zeroV1 = true;
            }

            float f = transacc[0];

            if (!Float.isNaN(f)) {
                if (ptAcc != 0) dt = (etime - ptAcc) / N2S;
                ptAcc = etime;
                Log.i(TAG, "处理后加速度: "+transacc[0]+",  "+ transacc[1]+",  "+transacc[2]);
                tranamx.set(0, transacc[0]);
                tranamx.set(1, transacc[1]);
                tranamx.set(2, transacc[2]);

//                sums = sums.plus(dt, tranamx);
//
//                dis = dis.plus(dt, sums);


            }
            dataLogger.setAcceleration(transacc);
//            dataLogger.setV(sums);
//            dataLogger.setDis(dis);

            hasAcceleration = true;
        } else if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
            if (ptMag != 0) dt = (etime - ptMag) / N2S;
            ptMag = etime;

            System.arraycopy(event.values, 0, magnetic, 0, event.values.length);
            if ((magnetic[0]*magnetic[0]+magnetic[1]*magnetic[1]+magnetic[2]*magnetic[2])<400)
                return;
            hasMagnetic = true;
        } else if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {

            if (gyroUBias.calcBias(event.values)){
                float[] gyBias = gyroUBias.getBias();
                if (ptGyro != 0) dt = (etime - ptGyro) / N2S;
                ptGyro = etime;

                System.arraycopy(event.values, 0, rotation, 0, event.values.length);
                rotation[0] = rotation[0] - gyBias[0];
                rotation[1] = rotation[1] - gyBias[1];
                rotation[2] = rotation[2] - gyBias[2];

                if ((rotation[0] * rotation[0] + rotation[1] * rotation[1] + rotation[2] * rotation[2]) < 0.5){
                    zeroV2 = true;
                }


                switch (mode) {
                    case GYROSCOPE_ONLY:
                        if (!orientationGyroscope.isBaseOrientationSet()) {
                            orientationGyroscope.setBaseOrientation(Quaternion.IDENTITY);
                        } else {
                            //TODO: timestamp?
                            fusedOrientation = orientationGyroscope.calculateOrientation(rotation, event.timestamp);
                        }
                        break;
                    case COMPLIMENTARY_FILTER:
                        if (!orientationComplimentaryFusion.isBaseOrientationSet()) {
                            if (hasAcceleration && hasMagnetic) {
                                orientationComplimentaryFusion.setBaseOrientation(RotationUtil.getOrientationQuaternionFromAccelerationMagnetic(acceleration, magnetic));
                            }
                        } else {
                            //这里得到的就是航向、俯仰、横滚了
                            fusedOrientation = orientationComplimentaryFusion.calculateFusedOrientation(rotation, event.timestamp, acceleration, magnetic);
                        }

                        break;
                    case KALMAN_FILTER:

                        if (!orientationKalmanFusion.isBaseOrientationSet()) {
                            if (hasAcceleration && hasMagnetic) {
                                orientationKalmanFusion.setBaseOrientation(RotationUtil.getOrientationQuaternionFromAccelerationMagnetic(acceleration, magnetic));
                            }
                        } else {
                            fusedOrientation = orientationKalmanFusion.calculateFusedOrientation(rotation, event.timestamp, acceleration, magnetic);
                        }
                        break;
                }

                if (meanFilterEnabled) {
                    fusedOrientation = meanFilter.filter(fusedOrientation);
                }
//                degret[0] = (float) ((Math.toDegrees(fusedOrientation[0]) + 360)%360);
//                degret[1] = (float) ((-(Math.toDegrees(fusedOrientation[1]))) % 180);
//                degret[2] = (float) (-(Math.toDegrees(fusedOrientation[2])));

                degret[0] = (float) (Math.toDegrees(fusedOrientation[0]));
                degret[1] = (float) (Math.toDegrees(fusedOrientation[1]));
                degret[2] = (float) (Math.toDegrees(fusedOrientation[2]));


                dataLogger.setRotation(degret);

                tntob = mTrans.cbt(degret[0], degret[1], degret[2]);
            }

        }

        if (zeroV1 & zeroV2 & zeroV3){
            clr_v();
        }

        //TODO: 应该先转方向的！方向一直在变啊是不是撒！
        mA = tranamx.mult(tntob);
        dataLogger.setTA(mA);
        Log.i(TAG, "-------------------------:  "+dt+"  :------------------------");
        Log.i(TAG, "东北天加速度: "+mA.get(0)+",  "+ mA.get(1)+",  "+mA.get(2));
        if(Math.abs(getR(mA) - getR(mA_pre))< 0.1){
            mV = mV_pre;
        }
        mV = sums(dt,mA,mV_pre);
//        mV = mV.plus(dt, mA);
        Log.i(TAG, "东北天速度: "+mV.get(0)+",  "+ mV.get(1)+",  "+mV.get(2));
        mX = sums(dt,mV,mX);
        delX = sums(dt,mV);

        mA_pre = mA;
        mV_pre = mV;
        if (getR(mA_pre) < 0.3){
            zeroV3 = true;
        }

        Log.i("lxy", "东北天位移cha:   " +delX.get(0)+",  "+ delX.get(1)+",  "+delX.get(2));
//        mX = mX.plus(dt, mV);
        dataLogger.setV(mV);
        Log.i("lxy", "东北天位移:   " +mX.get(0)+",  "+ mX.get(1)+",  "+mX.get(2));
        dataLogger.setDis(mX);
        dataLogger.setResult(delX);
        setTranstoGPX(mX.get(0), mX.get(1), mX.get(2));

//        nacc = dis.mult(tntob);
//        dataLogger.setNacc(nacc);
        /**
         * float nflag = (float) mX.get(1);
         if (!Float.isNaN(nflag)) {

         //TODO：mX已经很大了，乘个矩阵要疯了

         result = mX.mult(ntoe);

         dataLogger.setResult(result);

         setTranstoGPX(result.get(0), result.get(1), result.get(2));
         }
         */



    }

    private double getR(SimpleMatrix a_pre) {
        double r = a_pre.get(0) * a_pre.get(0) + a_pre.get(1)*a_pre.get(1)+a_pre.get(2)*a_pre.get(2);
        r = Math.sqrt(r);
        return r;
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {
    }


    private boolean getPrefMeanFilterEnabled() {
        SharedPreferences prefs = PreferenceManager
                .getDefaultSharedPreferences(getApplicationContext());

        return prefs.getBoolean(ConfigActivity.MEAN_FILTER_SMOOTHING_ENABLED_KEY,
                false);
    }

    private float getPrefMeanFilterTimeConstant() {
        SharedPreferences prefs = PreferenceManager
                .getDefaultSharedPreferences(getApplicationContext());

        return Float.valueOf(prefs.getString(ConfigActivity.MEAN_FILTER_SMOOTHING_TIME_CONSTANT_KEY, "0.5"));
    }

    private boolean getPrefKalmanEnabled() {
        SharedPreferences prefs = PreferenceManager
                .getDefaultSharedPreferences(getApplicationContext());

        return prefs.getBoolean(ConfigActivity.KALMAN_QUATERNION_ENABLED_KEY,
                false);
    }

    private boolean getPrefComplimentaryEnabled() {
        SharedPreferences prefs = PreferenceManager
                .getDefaultSharedPreferences(getApplicationContext());

        return prefs.getBoolean(ConfigActivity.COMPLIMENTARY_QUATERNION_ENABLED_KEY,
                false);
    }

    private float getPrefImuOCfQuaternionCoeff() {
        SharedPreferences prefs = PreferenceManager
                .getDefaultSharedPreferences(getApplicationContext());

        return Float.valueOf(prefs.getString(
                ConfigActivity.COMPLIMENTARY_QUATERNION_COEFF_KEY, "0.5"));
    }


    /**
     * Initialize the UI.
     */
    private void initUI() {
        Toolbar toolbar = findViewById(R.id.toolbar);
        setSupportActionBar(toolbar);
        if (getSupportActionBar() != null) {
            getSupportActionBar().setDisplayShowHomeEnabled(true);
        }


        tvXAxis = this.findViewById(R.id.value_x_axis_calibrated);
        tvYAxis = this.findViewById(R.id.value_y_axis_calibrated);
        tvZAxis = this.findViewById(R.id.value_z_axis_calibrated);

        LineX = this.findViewById(R.id.linx);
        LineY = this.findViewById(R.id.liny);
        AX = this.findViewById(R.id.AX);
        AY = this.findViewById(R.id.AY);
        VX = this.findViewById(R.id.VX);
        VY = this.findViewById(R.id.VY);

    }

    private void reset() {

        switch (mode) {
            case GYROSCOPE_ONLY:
                orientationGyroscope.reset();
                break;
            case COMPLIMENTARY_FILTER:
                orientationComplimentaryFusion.reset();
                break;
            case KALMAN_FILTER:
                orientationKalmanFusion.reset();
                break;
        }

        acceleration = new float[3];
        magnetic = new float[3];

        hasAcceleration = false;
        hasMagnetic = false;

        handler = new Handler();

        runable = new Runnable() {
            @Override
            public void run() {
                handler.postDelayed(this, 100);
                updateText();
            }
        };
    }

    private void readPrefs() {
        meanFilterEnabled = getPrefMeanFilterEnabled();
        complimentaryFilterEnabled = getPrefComplimentaryEnabled();
        kalmanFilterEnabled = getPrefKalmanEnabled();

        if (meanFilterEnabled) {
            meanFilter.setTimeConstant(getPrefMeanFilterTimeConstant());
        }

        if (!complimentaryFilterEnabled && !kalmanFilterEnabled) {
            mode = Mode.GYROSCOPE_ONLY;
        } else if (complimentaryFilterEnabled) {
            mode = Mode.COMPLIMENTARY_FILTER;
        } else if (kalmanFilterEnabled) {
            mode = Mode.KALMAN_FILTER;
        }
    }

    private void showHelpDialog() {
        Dialog helpDialog = new Dialog(this);

        helpDialog.setCancelable(true);
        helpDialog.setCanceledOnTouchOutside(true);
        helpDialog.requestWindowFeature(Window.FEATURE_NO_TITLE);

        View view = getLayoutInflater()
                .inflate(R.layout.layout_help_home, null);

        helpDialog.setContentView(view);

        helpDialog.show();
    }


    @SuppressLint({"DefaultLocale", "SetTextI18n"})
    private void updateText() {
        tvXAxis.setText(String.format("%.2f", (Math.toDegrees(fusedOrientation[0])) ));
        tvYAxis.setText(String.format("%.2f", (Math.toDegrees(fusedOrientation[1])) ));
        tvZAxis.setText(String.format("%.2f", (Math.toDegrees(fusedOrientation[2])) ));
        LineX.setText("LX:"+String.format("%.2f", mX.get(0) ));
        LineY.setText("LY:"+String.format("%.2f", mX.get(1) ));

        AX.setText("AX:"+String.format("%.2f", mA.get(0) ));
        AY.setText("AY:"+String.format("%.2f", mA.get(1) ));

        VX.setText("VX:"+String.format("%.2f", mV.get(0) ));
        VY.setText("VY:"+String.format("%.2f", mV.get(1) ));

    }


    private void requestPermissions() {
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.WRITE_EXTERNAL_STORAGE) != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.WRITE_EXTERNAL_STORAGE}, WRITE_EXTERNAL_STORAGE_REQUEST);
        }
    }


    private enum Mode {
        GYROSCOPE_ONLY,
        COMPLIMENTARY_FILTER,
        KALMAN_FILTER
    }



    public void updateTrackPoint(double lat, double lon) {
        ((TextView) findViewById(R.id.lat)).setText(Double.valueOf(lat).toString());
        ((TextView) findViewById(R.id.lon)).setText(Double.valueOf(lon).toString());

    }

    public String gpxTrackPoint(double lat, double lon, double ele, long time) {
        String trkpt = "<trkpt";
        trkpt += " lon=\"" + Double.valueOf(lon).toString() + "\"";
        trkpt += " lat=\"" + Double.valueOf(lat).toString() + "\"";
        trkpt += ">\n  <ele>" + Double.valueOf(ele).toString() + "</ele>\n";
        byte timebytes[] = new Timestamp(time).toString().getBytes();
        timebytes[10] = 'T';
        timebytes[19] = 'Z';
        trkpt += "  <time>" + new String(timebytes).substring(0, 20) + "</time>\n";
        trkpt += "</trkpt>\n";
        return trkpt;
    }

    public void setTranstoGPX(double x, double y, double z) {

        /**
         * 这是真滴更新
         */
//        double a=6378137.00;            // 长半轴，全部参考WGS-84椭球参数表
//        double b=6356752.3142;          // 短半轴
//        double alfa=1/298.257223563;    // 扁率
//        double e2=0.00669437999013;     // 第一偏心率的平方
//
//        double L=longitude;             // 经度，即水平方向方位角
//        double H1,N1,B1;
//        double N0=a;
//        double H0=Math.sqrt(x*x+y*y+z*z)-Math.sqrt(a*b);
//        double B0=Math.atan(z/Math.sqrt(x*x+y*y)/(1.0-e2*N0/(N0+H0)));
//
//            N1 = a / Math.sqrt(1.0 - e2 * Math.pow(Math.sin(B0), 2));
//            H1 = Math.sqrt(x * x + y * y) / Math.cos(B0) - N0;
//            B1 = Math.atan(z / Math.sqrt(x * x + y * y) / (1.0 - e2 * N1 / (N1 + H1)));
//
//            if (Math.abs(H1 - H0) > 0.001 & Math.abs(B1 - B0) > 4.848132257047973e-11) {
//                N0 = N1;
//                H0 = H1;
//                B0 = B1;
//            }
//
//        longitude = L;
//        latitude =
//        double perlon = 2 * Math.PI * Rearth / 360 ;
//        double perlat = 2 * Math.PI * Rearth * Math.cos(latitude)/360;
        double perlon = Rearth;
        double perlat = Rearth * Math.cos(latitude);
        x = x / perlon;
        y = y / perlat;
        latitude = initLatitude + y;
        longitude = initLongitude + x;

        Log.i(TAG, "setTranstoGPX: " + longitude + "," + latitude);
        ntoe = mTrans.eton(longitude, latitude);
        updateTrackPoint(latitude, longitude);
        //ntoe = mTrans.eton(mLocation.getLongitude()+x,mLocation.getLatitude()+y);

        if (gpxLogWriter == null)
            return;
        try {
            i++;

            if (i > numbers){
                GPXEntry mGPXEntry = new GPXEntry(latitude,longitude,ele,ptAcc);
                mGPXEntries.add(mGPXEntry);

                Log.i(TAG, "mGPX: "+mGPXEntries.size());
                try {
                    if (mGPXEntries.size()%50 == 0 ){
                        matcher(mGPXEntries);
                        clr_all();
                        Toast.makeText(GyroscopeActivity.this,initLatitude+",  "+initLongitude,Toast.LENGTH_SHORT).show();
                }

                }catch (Exception e){
                    Toast.makeText(GyroscopeActivity.this,e.getMessage(),Toast.LENGTH_SHORT).show();
                }
                gpxLogWriter.append(gpxTrackPoint(latitude, longitude, ele, ptAcc));
                i = 0;
            }
        } catch (Exception e) {
            gpxLogWriter = null;
        }
    }

    @SuppressLint("MissingPermission")

    static String xmlHeader = "<?xml version='1.0' encoding='Utf-8' standalone='yes' ?>";
    static String gpxTrackHeader = "<gpx xmlns=\"http://www.topografix.com/GPX/1/0\" version=\"1.0\" creator=\"org.yriarte.tracklogger\">\n<trk>\n<trkseg>\n";
    static String gpxTrackFooter = "\n</trkseg>\n</trk>\n</gpx>\n";

    public void startLogging() {
        if (gpxLogWriter != null)
            stopLogging();
        if (!Environment.MEDIA_MOUNTED.equals(Environment.getExternalStorageState()))
            return;
        try {
            //+"_" + String.valueOf(Calendar.getInstance().getTime().getTime())+
            gpxLogWriter = new FileWriter(Environment.getExternalStorageDirectory().getAbsolutePath() + "/" + getString(R.string.app_name) + ".gpx");
            gpxLogWriter.write(xmlHeader + gpxTrackHeader);
        } catch (Exception e) {
            gpxLogWriter = null;
        }

    }

    public void stopLogging() {
        if (gpxLogWriter == null)
            return;
        try {
            gpxLogWriter.append(gpxTrackFooter);
            gpxLogWriter.close();
        } catch (Exception e) {
        }
        gpxLogWriter = null;

    }


    public void matcher(ArrayList<GPXEntry> gpxEntries){
        GraphHopper hopper = new GraphHopperOSM();
        hopper.setGraphHopperLocation("/mnt/sdcard/Download/graph-cache");
        CarFlagEncoder encoder = new CarFlagEncoder();
        hopper.setEncodingManager(new EncodingManager(encoder));
        hopper.getCHFactoryDecorator().setEnabled(false);
        hopper.setMinNetworkSize(4,2);
        hopper.importOrLoad();

        String algorithm = Parameters.Algorithms.DIJKSTRA_BI;
        Weighting weighting = new FastestWeighting(encoder);
        AlgorithmOptions algoOptions = new AlgorithmOptions(algorithm, weighting);

        MapMatching mapMatching = new MapMatching(hopper, algoOptions);
        MatchResult mr = mapMatching.doWork(gpxEntries);


        List<EdgeMatch> matches = mr.getEdgeMatches();

        Toast.makeText(GyroscopeActivity.this,"Matched!",Toast.LENGTH_SHORT).show();

        initLongitude = matches.get(matches.size()-1).getGpxExtensions().get(0).getEntry().getLon();
        initLatitude = matches.get(matches.size()-1).getGpxExtensions().get(0).getEntry().getLat();

    }

    private SimpleMatrix sums(double dt,SimpleMatrix m,SimpleMatrix sums){
        SimpleMatrix results = sums.copy();
        results.set(0,sums.get(0) + dt * m.get(0));
        results.set(1,sums.get(1) + dt * m.get(1));
        results.set(2,sums.get(2) + dt * m.get(2));
        return results;
    }

    private SimpleMatrix sums(double dt,SimpleMatrix m){
        SimpleMatrix results = m.copy();
        results.set(0,dt * m.get(0));
        results.set(1,dt * m.get(1));
        results.set(2,dt * m.get(2));
        return results;
    }


}
