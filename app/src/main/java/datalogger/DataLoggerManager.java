package datalogger;

import android.content.Context;
import android.os.Environment;

import org.ejml.data.SimpleMatrix;

import java.io.File;
import java.util.ArrayList;
import java.util.Calendar;

public class DataLoggerManager implements Runnable {
    private static final String tag = DataLoggerManager.class.getSimpleName();

    public final static String DEFAULT_APPLICATION_DIRECTORY = "lxy";

    private final static long THREAD_SLEEP_TIME = 20;
    public final static String FILE_NAME_SEPARATOR = "-";

    // boolean to indicate if the data should be written to a file.
    private volatile boolean logData = false;

    // Log output time stamp
    protected long logTime = 0;

    private ArrayList<String> csvHeaders;
    private ArrayList<String> csvValues1;
    private ArrayList<String> csvValues2;
    private ArrayList<String> csvValues3;
    private ArrayList<String> csvValues4;
    private ArrayList<String> csvValues5;
    private ArrayList<String> csvValues6;

    private DataLoggerInterface dataLogger;

    private Context context;

    private volatile ArrayList<String> rotation;
    private volatile ArrayList<String> acceleration;
    private volatile ArrayList<String> sums;
    private volatile ArrayList<String> ta;
    private volatile ArrayList<String> dis;
    private volatile ArrayList<String> result;

    private Thread thread;

    public DataLoggerManager(Context context) {
        this.context = context;
        csvHeaders = getCsvHeaders();
        csvValues1 = new ArrayList<>();
        csvValues2 = new ArrayList<>();
        csvValues3 = new ArrayList<>();
        csvValues4 = new ArrayList<>();
        csvValues5 = new ArrayList<>();
        csvValues6 = new ArrayList<>();
        rotation = new ArrayList<>();
        acceleration = new ArrayList<>();
        sums = new ArrayList<>();
        ta = new ArrayList<>();
        dis = new ArrayList<>();
        result = new ArrayList<>();

    }

    @Override
    public void run() {
        while (logData && !Thread.currentThread().isInterrupted()) {
            // Check if the row is filled and ready to be written to the
            // log.
            logData();

            try {
                Thread.sleep(THREAD_SLEEP_TIME);
            } catch (InterruptedException e) {
                // very important to ensure the thread is killed
                Thread.currentThread().interrupt();
            }
        }

        // very important to ensure the thread is killed
        Thread.currentThread().interrupt();
    }


    public void startDataLog() throws IllegalStateException {
        if (!logData) {
            logData = true;
            logTime = System.currentTimeMillis();
            dataLogger = new CsvDataLogger(context, getFile(this.getFilePath(), this.getFileName()));
            dataLogger.setHeaders(csvHeaders);
            thread = new Thread(this);
            thread.start();
        } else {
            throw new IllegalStateException("Logger is already started!");
        }
    }

    public String stopDataLog() throws IllegalStateException {
        if (logData) {
            logData = false;
            thread.interrupt();
            thread = null;
            return dataLogger.writeToFile();
        }else {
            throw new IllegalStateException("Logger is already stopped!");
        }
    }

    public void setRotation(float[] rotation) {
        if(rotation != null) {
            synchronized (rotation) {
                this.rotation.clear();
                for (int i = 0; i < 3; i++) {
                    this.rotation.add(String.valueOf(rotation[i]));
                }
            }
        }
    }

    public void setAcceleration(float[] acceleration) {
        synchronized (acceleration) {
            this.acceleration.clear();
            for (int i = 0; i < 3; i++) {
                this.acceleration.add(String.valueOf(acceleration[i]));
            }
        }
    }

    public void setV(SimpleMatrix mV) {
        synchronized (sums) {
            this.sums.clear();
            for (int i = 0; i < 3; i++) {
                this.sums.add(String.valueOf(mV.get(i)));
            }
        }
    }

    public void setTA(SimpleMatrix ta) {
        synchronized (ta) {
            this.ta.clear();
            for (int i = 0; i < 3; i++) {
                this.ta.add(String.valueOf(ta.get(i)));
            }
        }
    }

    public void setDis(SimpleMatrix dis) {
        synchronized (dis) {
            this.dis.clear();
            for (int i = 0; i < 3; i++) {
                this.dis.add(String.valueOf(dis.get(i)));
            }
        }
    }

    public void setResult(SimpleMatrix result) {
        synchronized (result) {
            this.result.clear();
            for (int i = 0; i < 3; i++) {
                this.result.add(String.valueOf(result.get(i)));
            }
        }
    }

    private void logData() {
        double currt = (System.currentTimeMillis() - logTime) / 1000.0f;
        csvValues1.clear();
        csvValues1.add(String.valueOf(currt));
        csvValues2.clear();
        csvValues2.add(String.valueOf(currt));
        csvValues3.clear();
        csvValues3.add(String.valueOf(currt));
        csvValues4.clear();
        csvValues4.add(String.valueOf(currt));
        csvValues5.clear();
        csvValues5.add(String.valueOf(currt));
        csvValues6.clear();
        csvValues6.add(String.valueOf(currt));

        synchronized (rotation) {
            csvValues1.addAll(rotation);
        }
        dataLogger.addRow(csvValues1);

        synchronized (acceleration) {
            csvValues2.addAll(acceleration);
        }

        dataLogger.addRow(csvValues2);

        synchronized (ta) {
            csvValues3.addAll(ta);
        }

        dataLogger.addRow(csvValues3);

        synchronized (sums) {
            csvValues4.addAll(sums);
        }

        dataLogger.addRow(csvValues4);

        synchronized (dis) {
            csvValues5.addAll(dis);
        }

        dataLogger.addRow(csvValues5);

        synchronized (result) {
            csvValues6.addAll(result);
        }

        dataLogger.addRow(csvValues6);
    }



    private File getFile(String filePath, String fileName) {
        File dir = new File(filePath);

        if (!dir.exists()) {
            dir.mkdirs();
        }

        return new File(dir, fileName);
    }

    private String getFilePath() {
        return new StringBuilder().append(Environment.getExternalStorageDirectory()).append(File.separator).append
                (DEFAULT_APPLICATION_DIRECTORY).append(File.separator).toString();
    }

    private String getFileName() {
        Calendar c = Calendar.getInstance();

        return new StringBuilder().append(DEFAULT_APPLICATION_DIRECTORY).append(FILE_NAME_SEPARATOR)
                .append(c.get(Calendar.YEAR)).append(FILE_NAME_SEPARATOR).append(c.get(Calendar.MONTH) + 1).append
                        (FILE_NAME_SEPARATOR).
                        append(c.get(Calendar.DAY_OF_MONTH)).append(FILE_NAME_SEPARATOR).append(c.get(Calendar.HOUR))
                .append("-").append(c.get(Calendar.MINUTE)).append(FILE_NAME_SEPARATOR).append(c.get(Calendar.SECOND)
                ).append(".csv").toString();
    }

    private ArrayList<String> getCsvHeaders() {
        ArrayList<String> headers = new ArrayList<>();

        headers.add("Timestamp");
        headers.add("X");
        headers.add("Y");
        headers.add("Z");

        return headers;
    }
}
