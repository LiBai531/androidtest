package deadreckoning.activity;

import android.annotation.SuppressLint;
import android.annotation.TargetApi;
import android.app.Activity;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.LinearLayout;
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

import java.io.FileWriter;
import java.io.IOException;
import java.sql.Timestamp;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Calendar;
import java.util.List;

import deadreckoning.R;
import deadreckoning.extra.ExtraFunctions;
import deadreckoning.filewriting.DataFileWriter;
import deadreckoning.graph.ScatterPlot;
import deadreckoning.orientation.GyroscopeDeltaOrientation;
import deadreckoning.orientation.GyroscopeEulerOrientation;
import deadreckoning.orientation.MagneticFieldOrientation;
import deadreckoning.stepcounting.DynamicStepCounter;
import deadreckoning.tranc2gps.Tranc2Gps;
import matching.EdgeMatch;
import matching.MapMatching;
import matching.MatchResult;


public class GraphActivity extends Activity implements SensorEventListener, LocationListener {

    private static final long GPS_SECONDS_PER_WEEK = 511200L;

    private static final float GYROSCOPE_INTEGRATION_SENSITIVITY = 0.0025f;

    long Rearth = 111000; //m
    FileWriter gpxLogWriter;
    double initLongitude;
    double initLatitude;
    double initEle;
    double longitude;
    double latitude;
    double ele;
    long etime;
    private ArrayList<GPXEntry> mGPXEntries = new ArrayList<>();
    Location mLocation;
    private Tranc2Gps mTranc2Gps;

    private String TAG = "lxy";

    // for log
    private static final String FOLDER_NAME = "Dead_Reckoning/Graph_Activity";
    private static final String[] DATA_FILE_NAMES = {
            "Initial_Orientation",
            "Linear_Acceleration",
            "Gyroscope_Uncalibrated",
            "Magnetic_Field_Uncalibrated",
            "Gravity",
            "XY_Data_Set"
    };
    private static final String[] DATA_FILE_HEADINGS = {
            "Initial_Orientation",
            "Linear_Acceleration" + "\n" + "t;Ax;Ay;Az;findStep",
            "Gyroscope_Uncalibrated" + "\n" + "t;uGx;uGy;uGz;xBias;yBias;zBias;heading",
            "Magnetic_Field_Uncalibrated" + "\n" + "t;uMx;uMy;uMz;xBias;yBias;zBias;heading",
            "Gravity" + "\n" + "t;gx;gy;gz",
            "XY_Data_Set" + "\n" + "weekGPS;secGPS;t;strideLength;magHeading;gyroHeading;originalPointX;originalPointY;rotatedPointX;rotatedPointY"
    };

    private DynamicStepCounter dynamicStepCounter;
    private GyroscopeDeltaOrientation gyroscopeDeltaOrientation;
    private GyroscopeEulerOrientation gyroscopeEulerOrientation;
    private DataFileWriter dataFileWriter;
    private ScatterPlot scatterPlot;

    private Button buttonStart;
    private Button buttonStop;
    private Button buttonAddPoint;
    private LinearLayout mLinearLayout;

    private SensorManager sensorManager;
    private LocationManager locationManager;

    float[] gyroBias;
    float[] magBias;
    float[] currGravity; //current gravity
    float[] currMag; //current magnetic field

    private boolean isRunning;
    private boolean isCalibrated;
    private boolean usingDefaultCounter;
    private boolean areFilesCreated;
    private float strideLength;
    private float gyroHeading;
    private float magHeading;
    private float weeksGPS;
    private float secondsGPS;

    private long startTime;
    private boolean firstRun;

    private float initialHeading;

    @SuppressLint("MissingPermission")
    @TargetApi(Build.VERSION_CODES.KITKAT)
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_graph);

        //defining needed variables
        gyroBias = null;
        magBias = null;
        currGravity = null;
        currMag = null;
        mTranc2Gps = new Tranc2Gps();

        String counterSensitivity;

        isRunning = isCalibrated = usingDefaultCounter = areFilesCreated = false;
        firstRun = true;
        strideLength = 0;
        initialHeading = gyroHeading = magHeading = 0;
        weeksGPS = secondsGPS = 0;
        startTime = 0;

        //getting global settings
        strideLength =  getIntent().getFloatExtra("stride_length", 2.5f);
        isCalibrated = getIntent().getBooleanExtra("is_calibrated", false);
        gyroBias = getIntent().getFloatArrayExtra("gyro_bias");
        magBias = getIntent().getFloatArrayExtra("mag_bias");

        //using user_name to get index of user in userList, which is also the index of the user's stride_length
        counterSensitivity = UserListActivity.preferredStepCounterList
                .get(UserListActivity.userList.indexOf(getIntent().getStringExtra("user_name")));

        //usingDefaultCounter is counterSensitivity = "default" and sensor is available
        usingDefaultCounter = counterSensitivity.equals("default") &&
                getIntent().getBooleanExtra("step_detector", false);

        //initializing needed classes
        gyroscopeDeltaOrientation = new GyroscopeDeltaOrientation(GYROSCOPE_INTEGRATION_SENSITIVITY, gyroBias);
        if (usingDefaultCounter) //if using default TYPE_STEP_DETECTOR, don't need DynamicStepCounter
            dynamicStepCounter = null;
        else
            if (!counterSensitivity.equals("default"))
                dynamicStepCounter = new DynamicStepCounter(Double.parseDouble(counterSensitivity));
            else //if cannot use TYPE_STEP_DETECTOR but sensitivity = "default", use 1.0 sensitivity until user calibrates
                dynamicStepCounter = new DynamicStepCounter(1.0);

        //defining views
        buttonStart = (Button) findViewById(R.id.buttonGraphStart);
        buttonStop = (Button) findViewById(R.id.buttonGraphStop);
        buttonAddPoint = (Button) findViewById(R.id.buttonGraphClear);
        mLinearLayout = (LinearLayout) findViewById(R.id.linearLayoutGraph);

        //setting up graph with origin
        scatterPlot = new ScatterPlot("Position");
        scatterPlot.addPoint(0, 0);
        mLinearLayout.addView(scatterPlot.getGraphView(getApplicationContext()));

        //message user w/ user_name and stride_length info
        Toast.makeText(GraphActivity.this, "Stride Length: " + strideLength, Toast.LENGTH_SHORT).show();

        //starting GPS location tracking
        locationManager = (LocationManager) getSystemService(LOCATION_SERVICE);
        locationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 0, 0, GraphActivity.this);
        mLocation = locationManager.getLastKnownLocation(LocationManager.GPS_PROVIDER);

        initLongitude = mLocation.getLongitude();
        initLatitude = mLocation.getLatitude();
        initEle = mLocation.getAltitude();
        Toast.makeText(GraphActivity.this,"initlon:"+initLongitude+",  initlat"+initLatitude, Toast.LENGTH_SHORT).show();

        //starting sensors
        sensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
        sensorManager.registerListener(GraphActivity.this,
                sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY),
                SensorManager.SENSOR_DELAY_FASTEST);

        if (isCalibrated) {
            sensorManager.registerListener(GraphActivity.this,
                    sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED),
                    SensorManager.SENSOR_DELAY_FASTEST);
            sensorManager.registerListener(GraphActivity.this,
                    sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED),
                    SensorManager.SENSOR_DELAY_FASTEST);
        } else {
            sensorManager.registerListener(GraphActivity.this,
                    sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD),
                    SensorManager.SENSOR_DELAY_FASTEST);
            sensorManager.registerListener(GraphActivity.this,
                    sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE),
                    SensorManager.SENSOR_DELAY_FASTEST);
        }

        if (usingDefaultCounter) {
            sensorManager.registerListener(GraphActivity.this,
                    sensorManager.getDefaultSensor(Sensor.TYPE_STEP_DETECTOR),
                    SensorManager.SENSOR_DELAY_FASTEST);
        } else {
            sensorManager.registerListener(GraphActivity.this,
                    sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION),
                    SensorManager.SENSOR_DELAY_FASTEST);
        }

        //setting up buttons
        buttonStart.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                startLogging();

                isRunning = true;

                createFiles();

                if (usingDefaultCounter)
                    dataFileWriter.writeToFile("Linear_Acceleration",
                            "TYPE_LINEAR_ACCELERATION will not be recorded, since the TYPE_STEP_DETECTOR is being used instead."
                    );

                float[][] initialOrientation = MagneticFieldOrientation.getOrientationMatrix(currGravity, currMag, magBias);
                initialHeading = MagneticFieldOrientation.getHeading(currGravity, currMag, magBias);

                //saving initial orientation data
                dataFileWriter.writeToFile("Initial_Orientation", "init_Gravity: " + Arrays.toString(currGravity));
                dataFileWriter.writeToFile("Initial_Orientation", "init_Mag: " + Arrays.toString(currMag));
                dataFileWriter.writeToFile("Initial_Orientation", "mag_Bias: " + Arrays.toString(magBias));
                dataFileWriter.writeToFile("Initial_Orientation", "gyro_Bias: " + Arrays.toString(gyroBias));
                dataFileWriter.writeToFile("Initial_Orientation", "init_Orientation: " + Arrays.deepToString(initialOrientation));
                dataFileWriter.writeToFile("Initial_Orientation", "init_Heading: " + initialHeading);

                Log.d("init_heading", "" + initialHeading);

                //TODO: fix rotation matrix
                //gyroscopeEulerOrientation = new GyroscopeEulerOrientation(initialOrientation);

                gyroscopeEulerOrientation = new GyroscopeEulerOrientation(ExtraFunctions.IDENTITY_MATRIX);

                dataFileWriter.writeToFile("XY_Data_Set", "Initial_orientation: " +
                        Arrays.deepToString(initialOrientation));
                dataFileWriter.writeToFile("Gyroscope_Uncalibrated", "Gyroscope_bias: " +
                        Arrays.toString(gyroBias));
                dataFileWriter.writeToFile("Magnetic_Field_Uncalibrated", "Magnetic_field_bias:" +
                        Arrays.toString(magBias));

                buttonStart.setEnabled(false);
                buttonAddPoint.setEnabled(true);
                buttonStop.setEnabled(true);

            }
        });

        buttonStop.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                stopLogging();

                firstRun = true;
                isRunning = false;

                buttonStart.setEnabled(true);
                buttonAddPoint.setEnabled(true);
                buttonStop.setEnabled(false);

            }
        });

        buttonAddPoint.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                //complimentary filter
                float compHeading = ExtraFunctions.calcCompHeading(magHeading, gyroHeading);

//TODO:1

                Log.d("comp_heading", "" + compHeading);

                //getting and rotating the previous XY points so North 0 on unit circle
                float oPointX = scatterPlot.getLastYPoint();
                float oPointY = -scatterPlot.getLastXPoint();

                //calculating XY points from heading and stride_length
                oPointX += ExtraFunctions.getXFromPolar(strideLength, compHeading);
                oPointY += ExtraFunctions.getYFromPolar(strideLength, compHeading);

                //rotating points by 90 degrees, so north is up
                float rPointX = -oPointY;
                float rPointY = oPointX;

                //TODO:rX,rY
                setTranstoGPX(rPointX,rPointY);

                scatterPlot.addPoint(rPointX, rPointY);

                mLinearLayout.removeAllViews();
                mLinearLayout.addView(scatterPlot.getGraphView(getApplicationContext()));

            }
        });

    }

    @Override
    protected void onStop() {
        super.onStop();
        sensorManager.unregisterListener(this);
        locationManager.removeUpdates(this);
    }

    @SuppressLint("MissingPermission")
    @Override
    protected void onResume() {
        super.onResume();

        if (isRunning) {

            locationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 0, 0, GraphActivity.this);

            if (isCalibrated) {
                sensorManager.registerListener(GraphActivity.this,
                        sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED),
                        SensorManager.SENSOR_DELAY_FASTEST);
                sensorManager.registerListener(GraphActivity.this,
                        sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED),
                        SensorManager.SENSOR_DELAY_FASTEST);
            } else {
                sensorManager.registerListener(GraphActivity.this,
                        sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD),
                        SensorManager.SENSOR_DELAY_FASTEST);
                sensorManager.registerListener(GraphActivity.this,
                        sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE),
                        SensorManager.SENSOR_DELAY_FASTEST);
            }

            if (usingDefaultCounter) {
                sensorManager.registerListener(GraphActivity.this,
                        sensorManager.getDefaultSensor(Sensor.TYPE_STEP_DETECTOR),
                        SensorManager.SENSOR_DELAY_FASTEST);
            } else {
                sensorManager.registerListener(GraphActivity.this,
                        sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION),
                        SensorManager.SENSOR_DELAY_FASTEST);
            }

            buttonStart.setEnabled(false);
            buttonAddPoint.setEnabled(true);
            buttonStop.setEnabled(true);

        } else {

            buttonStart.setEnabled(true);
            buttonAddPoint.setEnabled(true);
            buttonStop.setEnabled(false);

        }

    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {}

    @Override
    public void onSensorChanged(SensorEvent event) {
        etime = event.timestamp;

        if(firstRun) {
            startTime = event.timestamp;
            firstRun = false;
        }

        if (event.sensor.getType() == Sensor.TYPE_GRAVITY) {
            currGravity = event.values;
            Log.d("gravity_values", Arrays.toString(event.values));
        } else if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD ||
                event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED) {
            currMag = event.values;
            Log.d("mag_values", Arrays.toString(event.values));
        }

        if (isRunning) {
            if (event.sensor.getType() == Sensor.TYPE_GRAVITY) {
                ArrayList<Float> dataValues = ExtraFunctions.arrayToList(event.values);
                dataValues.add(0, (float)(event.timestamp - startTime));
                dataFileWriter.writeToFile("Gravity", dataValues);
            } else if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD || event.sensor.getType() ==
                    Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED) {

                magHeading = MagneticFieldOrientation.getHeading(currGravity, currMag, magBias);

                Log.d("mag_heading", "" + magHeading);

                //saving magnetic field data
                ArrayList<Float> dataValues = ExtraFunctions.createList(
                        event.values[0], event.values[1], event.values[2],
                        magBias[0], magBias[1], magBias[2]
                );
                dataValues.add(0, (float)(event.timestamp - startTime));
                dataValues.add(magHeading);
                dataFileWriter.writeToFile("Magnetic_Field_Uncalibrated", dataValues);

            } else if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE ||
                    event.sensor.getType() == Sensor.TYPE_GYROSCOPE_UNCALIBRATED) {

                float[] deltaOrientation = gyroscopeDeltaOrientation.calcDeltaOrientation(event.timestamp, event.values);

                gyroHeading = gyroscopeEulerOrientation.getHeading(deltaOrientation);
                gyroHeading += initialHeading;

                Log.d("gyro_heading", "" + gyroHeading);

                //saving gyroscope data
                ArrayList<Float> dataValues = ExtraFunctions.createList(
                        event.values[0], event.values[1], event.values[2],
                        gyroBias[0], gyroBias[1], gyroBias[2]
                );
                dataValues.add(0, (float)(event.timestamp - startTime));
                dataValues.add(gyroHeading);
                dataFileWriter.writeToFile("Gyroscope_Uncalibrated", dataValues);

            } else if (event.sensor.getType() == Sensor.TYPE_LINEAR_ACCELERATION) {

                float norm = ExtraFunctions.calcNorm(
                        event.values[0] +
                        event.values[1] +
                        event.values[2]
                );

                //if step is found, findStep == true
                boolean stepFound = dynamicStepCounter.findStep(norm);

                if (stepFound) {

                    //saving linear acceleration data
                    ArrayList<Float> dataValues = ExtraFunctions.arrayToList(event.values);
                    dataValues.add(0, (float)(event.timestamp - startTime));
                    dataValues.add(1f);
                    dataFileWriter.writeToFile("Linear_Acceleration", dataValues);

                    //complimentary filter
                    float compHeading = ExtraFunctions.calcCompHeading(magHeading, gyroHeading);

                    //Log.d("comp_heading", "" + compHeading);

                    //getting and rotating the previous XY points so North 0 on unit circle
                    float oPointX = scatterPlot.getLastYPoint();
                    float oPointY = -scatterPlot.getLastXPoint();

                    //calculating XY points from heading and stride_length
                    oPointX += ExtraFunctions.getXFromPolar(strideLength, gyroHeading);
                    oPointY += ExtraFunctions.getYFromPolar(strideLength, gyroHeading);

                    //rotating points by 90 degrees, so north is up
                    float rPointX = -oPointY;
                    float rPointY = oPointX;

                    setTranstoGPX(rPointX,rPointY);

                    scatterPlot.addPoint(rPointX, rPointY);

                    //saving XY location data
                    dataFileWriter.writeToFile("XY_Data_Set",
                            weeksGPS,
                            secondsGPS,
                            (event.timestamp - startTime),
                            strideLength,
                            magHeading,
                            gyroHeading,
                            oPointX,
                            oPointY,
                            rPointX,
                            rPointY);

                    mLinearLayout.removeAllViews();
                    mLinearLayout.addView(scatterPlot.getGraphView(getApplicationContext()));

                    //if step is not found
                } else {
                    //saving linear acceleration data
                    ArrayList<Float> dataValues = ExtraFunctions.arrayToList(event.values);
                    dataValues.add(0, (float) event.timestamp);
                    dataValues.add(0f);
                    dataFileWriter.writeToFile("Linear_Acceleration", dataValues);
                }

            } else if (event.sensor.getType() == Sensor.TYPE_STEP_DETECTOR) {

                boolean stepFound = (event.values[0] == 1);

                if (stepFound) {

                    //complimentary filter
                    float compHeading = ExtraFunctions.calcCompHeading(magHeading, gyroHeading);

                    //Log.d("comp_heading", "" + compHeading);

                    //getting and rotating the previous XY points so North 0 on unit circle
                    float oPointX = scatterPlot.getLastYPoint();
                    float oPointY = -scatterPlot.getLastXPoint();

                    //calculating XY points from heading and stride_length
                    oPointX += ExtraFunctions.getXFromPolar(strideLength, gyroHeading);
                    oPointY += ExtraFunctions.getYFromPolar(strideLength, gyroHeading);

                    //rotating points by 90 degrees, so north is up
                    float rPointX = -oPointY;
                    float rPointY = oPointX;

                    setTranstoGPX(rPointX,rPointY);

                    scatterPlot.addPoint(rPointX, rPointY);

                    //saving XY location data
                    dataFileWriter.writeToFile("XY_Data_Set",
                            weeksGPS,
                            secondsGPS,
                            (event.timestamp - startTime),
                            strideLength,
                            magHeading,
                            gyroHeading,
                            oPointX,
                            oPointY,
                            rPointX,
                            rPointY);

                    mLinearLayout.removeAllViews();
                    mLinearLayout.addView(scatterPlot.getGraphView(getApplicationContext()));
                }

            }
        }

    }

    @Override
    public void onLocationChanged(Location location) {
        long GPSTimeSec = location.getTime() / 1000;
        weeksGPS = GPSTimeSec / GPS_SECONDS_PER_WEEK;
        secondsGPS = GPSTimeSec % GPS_SECONDS_PER_WEEK;
    }

    @Override
    public void onStatusChanged(String provider, int status, Bundle extras) {}

    @Override
    public void onProviderEnabled(String provider) {}

    @Override
    public void onProviderDisabled(String provider) {}

    private void createFiles() {
        if (!areFilesCreated) {
            try {
                dataFileWriter = new DataFileWriter(FOLDER_NAME, DATA_FILE_NAMES, DATA_FILE_HEADINGS);
            } catch (IOException e) {
                e.printStackTrace();
            }
            areFilesCreated = true;
        }
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

    public void setTranstoGPX(double x, double y) {

        //double perlat = 2 * Math.PI * Rearth * Math.cos(latitude)/360;
        double perlon = Rearth;
        double perlat = Rearth * Math.cos(latitude);
        x = x / perlon;
        y = y / perlat;
        latitude = initLatitude + y;
        longitude = initLongitude + x;

        Log.i("lxy", "setTranstoGPX: " + longitude + "," + latitude);

        //ntoe = mTrans.eton(mLocation.getLongitude()+x,mLocation.getLatitude()+y);

        if (gpxLogWriter == null)
            return;
        try {
            GPXEntry mGPXEntry = new GPXEntry(latitude,longitude,ele,etime);
            mGPXEntries.add(mGPXEntry);

            //Log.i(TAG, "mGPX: "+mGPXEntries.size());
//            try {
//                if (mGPXEntries.size()%20 == 0 ){
//                    matcher(mGPXEntries);
//                    clr_all();
//                    Toast.makeText(GraphActivity.this,initLatitude+",  "+initLongitude, Toast.LENGTH_SHORT).show();
//                }
//
//            }catch (Exception e){
//                Toast.makeText(GraphActivity.this,e.getMessage(), Toast.LENGTH_SHORT).show();
//            }
            gpxLogWriter.append(gpxTrackPoint(latitude, longitude, ele, etime));

        } catch (Exception e) {
            Toast.makeText(GraphActivity.this,e.getMessage(),Toast.LENGTH_SHORT).show();
            gpxLogWriter = null;
        }
    }

    @SuppressLint("MissingPermission")

    static String xmlHeader = "<?xml version='1.0' encoding='Utf-8' standalone='yes' ?>";
    static String gpxTrackHeader = "<gpx xmlns=\"http://www.topografix.com/GPX/1/0\" version=\"1.0\" creator=\"org.yriarte.tracklogger\">\n<trk>\n<trkseg>\n";
    static String gpxTrackFooter = "\n</trkseg>\n</trk>\n</gpx>\n";

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

        Toast.makeText(GraphActivity.this,"Matched!", Toast.LENGTH_SHORT).show();

        initLongitude = matches.get(matches.size()-1).getGpxExtensions().get(0).getEntry().getLon();
        initLatitude = matches.get(matches.size()-1).getGpxExtensions().get(0).getEntry().getLat();

    }

    public void startLogging() {
        if (gpxLogWriter != null)
            stopLogging();
        if (!Environment.MEDIA_MOUNTED.equals(Environment.getExternalStorageState()))
            return;
        try {
            //
            gpxLogWriter = new FileWriter(Environment.getExternalStorageDirectory().getAbsolutePath() + "/" + getString(R.string.app_name) +"_" + String.valueOf(Calendar.getInstance().getTime().getTime())+".gpx");
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

    private void clr_all(){
        scatterPlot.clearSet();
        mGPXEntries.clear();

    }



}
