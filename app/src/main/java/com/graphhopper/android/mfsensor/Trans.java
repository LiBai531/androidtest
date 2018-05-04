package com.graphhopper.android.mfsensor;

import org.ejml.data.SimpleMatrix;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

/**
 * Created by 83443 on 2018/4/3.
 */

public class Trans {

    public SimpleMatrix cem = new SimpleMatrix(3,3);
    public SimpleMatrix inv = new SimpleMatrix(3,3);

    public Trans(){

    }

    /**
     * 东北天到地球（inv）
     * @param lon
     * @param lat
     * @return
     */
    public SimpleMatrix eton(double lon,double lat){
        cem.set(0,0,-sin(lon));
        cem.set(0,1, cos(lon));
        cem.set(0,2,0);
        cem.set(1,0,-sin(lat)* cos(lon));
        cem.set(1,1,-sin(lon)* sin(lat));
        cem.set(1,2, cos(lat));
        cem.set(2,0, cos(lon)* cos(lat));
        cem.set(2,1, cos(lon)* cos(lat));
        cem.set(2,2, sin(lat));
        inv = cem.invert();


        return inv;

    }

    /**
     * 载体到东北天（inv）基于欧拉角
     * @param azimuth
     * @param pitch
     * @param roll
     * @return
     */
    public SimpleMatrix ntob(float azimuth,float pitch,float roll){
        cem.set(0,0,cos(roll)* cos(azimuth) + sin(azimuth)*sin(pitch)*sin(roll));
        cem.set(0,1, -cos(roll)*sin(azimuth)+sin(roll)*sin(pitch)*cos(azimuth));
        cem.set(0,2,-sin(roll)*cos(pitch));
        cem.set(1,0,cos(pitch)*sin(azimuth));
        cem.set(1,1,cos(pitch)*cos(azimuth));
        cem.set(1,2, sin(pitch));
        cem.set(2,0, sin(roll)*cos(azimuth)-cos(roll)*sin(pitch)*sin(azimuth));
        cem.set(2,1, -sin(roll)*cos(azimuth)-cos(roll)*sin(pitch)*cos(azimuth));
        cem.set(2,2, cos(roll)*cos(pitch));
        cem.print();
        inv = cem.invert();

        return inv;

    }

    public SimpleMatrix cbt(float azimuth,float pitch,float roll){
        cem.set(0,0,cos(azimuth)* cos(pitch));
        cem.set(0,1, -cos(azimuth)*sin(pitch)*sin(roll)-sin(azimuth)*cos(roll));
        cem.set(0,2,-cos(azimuth)*sin(pitch)*cos(roll)+sin(azimuth)*sin(roll));
        cem.set(1,0,cos(pitch)*sin(azimuth));
        cem.set(1,1,-sin(azimuth)*sin(pitch)*sin(roll)+cos(azimuth)*cos(roll));
        cem.set(1,2, -sin(azimuth)*sin(pitch)*cos(roll)-cos(azimuth)*sin(roll));
        cem.set(2,0, sin(pitch));
        cem.set(2,1, cos(pitch)*sin(roll));
        cem.set(2,2, cos(pitch)*cos(roll));
        cem.print();

        return cem;

    }
}
