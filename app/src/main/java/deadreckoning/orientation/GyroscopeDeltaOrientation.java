package deadreckoning.orientation;

import deadreckoning.extra.ExtraFunctions;

public class GyroscopeDeltaOrientation {

    private boolean isFirstRun;
    private float sensitivity;
    private float lastTimestamp;
    private float[] gyroBias;


    public GyroscopeDeltaOrientation() {
        this.gyroBias = new float[3];
        this.sensitivity = 0.0025f;
        this.isFirstRun = true;
    }

    public GyroscopeDeltaOrientation(float sensitivity, float[] gyroBias) {
        this();
        this.sensitivity = sensitivity;
        this.gyroBias = gyroBias;
    }

    /**
     * 计算角度
     * @param timestamp
     * @param rawGyroValues
     * @return
     */
    public float[] calcDeltaOrientation(long timestamp, float[] rawGyroValues) {
        //首次运行时间
        if (isFirstRun) {
            isFirstRun = false;
            lastTimestamp = ExtraFunctions.nsToSec(timestamp);
            return new float[3];
        }

        float[] unbiasedGyroValues = removeBias(rawGyroValues);

        //return deltaOrientation[]
        return integrateValues(timestamp, unbiasedGyroValues);
    }

    public void setBias(float[] gyroBias) {
        this.gyroBias = gyroBias;
    }

    /**
     * 原始值减偏差，小于阈值置零。
     * @param rawGyroValues
     * @return
     */
    private float[] removeBias(float[] rawGyroValues) {
        //忽略TYPE_UNCALIBRATED_GYROSCOPE的最后3个值，因为这只是Android计算的偏差
        float[] unbiasedGyroValues = new float[3];

        for (int i = 0; i < 3; i++)
            unbiasedGyroValues[i] = rawGyroValues[i] - gyroBias[i];

        //TODO: check how big of a difference this makes
        //高通滤波
        for (int i = 0; i < 3; i++)
            if (Math.abs(unbiasedGyroValues[i]) > sensitivity)
                unbiasedGyroValues[i] = unbiasedGyroValues[i];
            else
                unbiasedGyroValues[i] = 0;

        return unbiasedGyroValues;
    }

    /**
     * 角速度对时间积分
     * @param timestamp
     * @param gyroValues
     * @return
     */
    private float[] integrateValues(long timestamp, float[] gyroValues) {
        double currentTime = ExtraFunctions.nsToSec(timestamp);
        double deltaTime = currentTime - lastTimestamp;

        float[] deltaOrientation = new float[3];

        //积分角速度与时间的关系
        for (int i = 0; i < 3; i++)
            deltaOrientation[i] = gyroValues[i] * (float)deltaTime;

        lastTimestamp = (float) currentTime;

        return deltaOrientation;
    }

}
