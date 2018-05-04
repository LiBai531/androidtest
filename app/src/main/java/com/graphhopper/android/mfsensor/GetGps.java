package com.graphhopper.android.mfsensor;

import android.Manifest;
import android.annotation.SuppressLint;
import android.content.Context;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Bundle;
import android.os.Environment;
import android.support.v4.app.ActivityCompat;
import android.support.v7.app.AppCompatActivity;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import java.io.FileWriter;
import java.sql.Timestamp;
import java.util.Calendar;

import dbDao.DBManager;
import deadreckoning.R;


public class GetGps extends AppCompatActivity implements LocationListener {

    static int minTimeUpdateSeconds = 10;
    static float minDistanceUpdateMeters = 5;

    static String xmlHeader = "<?xml version='1.0' encoding='Utf-8' standalone='yes' ?>";
    static String gpxTrackHeader = "<gpx xmlns=\"http://www.topografix.com/GPX/1/0\" version=\"1.0\" creator=\"org.yriarte.tracklogger\">\n<trk>\n<trkseg>\n";
    static String gpxTrackFooter = "\n</trkseg>\n</trk>\n</gpx>\n";
    FileWriter gpxLogWriter;

    LocationManager mLocationManager;
    Location mLocation;

    public DBManager dbHelper;
    float v;
    double lan;
    double lon;
    double ele;
    float bearing;


//    /**
//     * 复制资源文件
//     * @param context
//     * @param desPath
//     * @param fileNmae
//     */
//    public static void copyAssetsFile(Context context, String desPath,String fileNmae) {
//        try {
//            int bytesum = 0;
//            int byteread = 0;
//            File desfile = new File(desPath+"/"+fileNmae);
//            if (!desfile.exists()) { //文件不存在时
//                InputStream inStream = context.getClass().getClassLoader()
//                        .getResourceAsStream("assets/" + fileNmae); //读入资源文件
//                FileOutputStream fs = new FileOutputStream(desPath+"/"+fileNmae);
//                byte[] buffer = new byte[4096];
//                int length;
//                while ( (byteread = inStream.read(buffer)) != -1) {
//                    bytesum += byteread; //字节数 文件大小
//                    fs.write(buffer, 0, byteread);
//                    fs.flush();
//                }
//                fs.close();
//                inStream.close();
//                Log.i("lxy", "数据库已拷贝！");
//            }
//        }
//        catch (Exception e) {
//            Log.i("lxy","复制单个文件操作出错");
//            e.printStackTrace();
//
//        }
//
//    }

//    @SuppressLint("MissingPermission")
//    public void startLogging() {
//        if (gpxLogWriter != null)
//            stopLogging();
//        mLocationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER,
//                minTimeUpdateSeconds * 1000,
//                minDistanceUpdateMeters,
//                this);
//        if (!Environment.MEDIA_MOUNTED.equals(Environment.getExternalStorageState()))
//            return;
//        try {
//            gpxLogWriter = new FileWriter(Environment.getExternalStorageDirectory().getAbsolutePath() + "/"
//                    + getString(R.string.app_name) + "_" + String.valueOf(Calendar.getInstance().getTime().getTime())
//                    + ".gpx"
//            );
//            gpxLogWriter.write(xmlHeader + gpxTrackHeader);
//        } catch (Exception e) {
//            gpxLogWriter = null;
//        }
//
//    }
//
//    public void stopLogging() {
//        mLocationManager.removeUpdates(this);
//        if (gpxLogWriter == null)
//            return;
//        try {
//            gpxLogWriter.append(gpxTrackFooter);
//            gpxLogWriter.close();
//        } catch (Exception e) {
//        }
//        gpxLogWriter = null;
//
//    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_track_logger);

        mLocationManager = (LocationManager) getApplicationContext().getSystemService(Context.LOCATION_SERVICE);

        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED && ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            Toast.makeText(GetGps.this,"Permission!",Toast.LENGTH_SHORT);
            return;
        }
        mLocation = mLocationManager.getLastKnownLocation(LocationManager.GPS_PROVIDER);
        v = mLocation.getSpeed();
//        if (mLocation == null)
//            return;
        /**
         * 初始
         */
//        updateTrackPoint(mLocation.getLatitude(),mLocation.getLongitude(),mLocation.getAltitude(),mLocation.getTime());

//        dbHelper = new DBManager(this);
//        dbHelper.openDatabase();
//        dbHelper.closeDatabase();



        Button b = findViewById(R.id.button);
        b.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                Intent intent = new Intent(GetGps.this,GyroscopeActivity.class);
                intent.putExtra("lon",120.15130282);
                intent.putExtra("lat",30.33422952);
                intent.putExtra("ele",65.0);
                intent.putExtra("v",v);
//                intent.putExtra("lon",lon);
//                intent.putExtra("lat",lan);
//                intent.putExtra("ele",ele);
                startActivity(intent);
            }
        });

    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {

        getMenuInflater().inflate(R.menu.menu_track_logger, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {

        int id = item.getItemId();

        if (id == R.id.action_log_start) {
//            startLogging();
            return true;
        }
        if (id == R.id.action_log_stop) {
//            stopLogging();
            return true;
        }

        return super.onOptionsItemSelected(item);
    }

//    public void updateTrackPoint(double lat, double lon, double ele, long time) {
//        ((TextView)findViewById(R.id.value_lat)).setText(Double.valueOf(lat).toString());
//        ((TextView)findViewById(R.id.value_lon)).setText(Double.valueOf(lon).toString());
//        ((TextView)findViewById(R.id.value_ele)).setText(Double.valueOf(ele).toString());
//        ((TextView)findViewById(R.id.value_time)).setText(new Timestamp(time).toString());
//    }

//    public String gpxTrackPoint(double lat, double lon, double ele, long time) {
//        String trkpt = "<trkpt";
//        trkpt += " lon=\"" + Double.valueOf(lon).toString() + "\"";
//        trkpt += " lat=\"" + Double.valueOf(lat).toString() + "\"";
//        trkpt += ">\n  <ele>" + Double.valueOf(ele).toString() + "</ele>\n";
//        byte timebytes[] = new Timestamp(time).toString().getBytes();
//        timebytes[10]='T'; timebytes[19]='Z';
//        trkpt += "  <time>" + new String(timebytes).substring(0,20) + "</time>\n";
//        trkpt += "</trkpt>\n";
//        return trkpt;
//    }

    @Override
    public void onLocationChanged(Location location) {
        v = mLocation.getSpeed();
        lan = mLocation.getLatitude();
        lon = mLocation.getLongitude();
        bearing = mLocation.getBearing(); // 偏离正北的度数
        ele = mLocation.getAltitude();


    }

    @Override
    public void onStatusChanged(String provider, int status, Bundle extras) {

    }

    @Override
    public void onProviderEnabled(String provider) {

    }

    @Override
    public void onProviderDisabled(String provider) {

    }

//    public void setTranstoGPX(double x,double y,double z){
//        //mLocation = location;
//        /**
//         * 这是真滴更新
//         */
//        updateTrackPoint(mLocation.getLatitude()+y, mLocation.getLongitude()+x, mLocation.getAltitude()+z, mLocation.getTime());
//        //ntoe = mTrans.eton(mLocation.getLongitude()+x,mLocation.getLatitude()+y);
//
//        if (gpxLogWriter == null)
//            return;
//        try {
//            gpxLogWriter.append(gpxTrackPoint(mLocation.getLatitude()+y, mLocation.getLongitude()+x, mLocation.getAltitude()+z, mLocation.getTime()));
//        } catch (Exception e)
//        {
//            gpxLogWriter = null;
//        }
//    }
}
