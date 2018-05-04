package deadreckoning.graph;

import android.content.Context;
import android.graphics.Color;

import org.achartengine.ChartFactory;
import org.achartengine.GraphicalView;
import org.achartengine.chart.PointStyle;
import org.achartengine.model.XYMultipleSeriesDataset;
import org.achartengine.model.XYSeries;
import org.achartengine.renderer.XYMultipleSeriesRenderer;
import org.achartengine.renderer.XYSeriesRenderer;

import java.util.ArrayList;

public class ScatterPlot {

    private String seriesName;
    private ArrayList<Double> xList;
    private ArrayList<Double> yList;

    public ScatterPlot (String seriesName) {
        this.seriesName = seriesName;
        xList = new ArrayList<>();
        yList = new ArrayList<>();
    }

    public GraphicalView getGraphView(Context context) {

        XYSeries mySeries;
        XYSeriesRenderer myRenderer;
        XYMultipleSeriesDataset myMultiSeries;
        XYMultipleSeriesRenderer myMultiRenderer;

        // 将ArrayList中的x轴数据添加到标准数组中
        double[] xSet = new double[xList.size()];
        for (int i = 0; i < xList.size(); i++)
            xSet[i] = xList.get(i);

        // ArrayList中的y轴数据添加到标准数组中
        double[] ySet = new double[yList.size()];
        for (int i = 0; i < yList.size(); i++)
            ySet[i] = yList.get(i);

        // 使用x轴和y轴数据创建一个新的序列
        mySeries = new XYSeries(seriesName);
        for (int i = 0; i < xSet.length; i++)
            mySeries.add(xSet[i], ySet[i]);

        // 定义图表视觉属性
        myRenderer = new XYSeriesRenderer();
        myRenderer.setFillPoints(true);
        myRenderer.setPointStyle(PointStyle.CIRCLE);
        myRenderer.setColor(Color.GREEN);

        myMultiSeries = new XYMultipleSeriesDataset();
        myMultiSeries.addSeries(mySeries);

        myMultiRenderer = new XYMultipleSeriesRenderer();
        myMultiRenderer.addSeriesRenderer(myRenderer);

        // 设置文本图形元素的大小
        myMultiRenderer.setPointSize(7); // 散点图点的大小
        myMultiRenderer.setShowLegend(false); // hide legend

        //set chart and label sizes
        myMultiRenderer.setChartTitle("Position");
        myMultiRenderer.setChartTitleTextSize(75);
        myMultiRenderer.setLabelsTextSize(40);

        //setting X labels and Y labels position
        int[] chartMargins = {100, 100, 25, 100}; //top, left, bottom, right
        myMultiRenderer.setMargins(chartMargins);
        myMultiRenderer.setYLabelsPadding(50);
        myMultiRenderer.setXLabelsPadding(10);

        //setting chart min/max
        double bound = getMaxBound();
        myMultiRenderer.setXAxisMin(-bound);
        myMultiRenderer.setXAxisMax(bound);
        myMultiRenderer.setYAxisMin(-bound);
        myMultiRenderer.setYAxisMax(bound);

        //returns the graphical view containing the graph
        return ChartFactory.getScatterChartView(context, myMultiSeries, myMultiRenderer);
    }

    //add a point to the series
    public void addPoint(double x, double y) {
        xList.add(x);
        yList.add(y);
    }

    public float getLastXPoint() {
        double x = xList.get(xList.size() - 1);
        return (float)x;
    }

    public float getLastYPoint() {
        double y = yList.get(yList.size() - 1);
        return (float)y;
    }

    public void clearSet() {
        xList.clear();
        yList.clear();
    }

    private double getMaxBound() {
        double max = 0;
        for (double num : xList)
            if (max < Math.abs(num))
                max = num;
        for (double num : yList)
            if (max < Math.abs(num))
                max = num;
        return (Math.abs(max) / 100) * 100 + 100; //rounding up to the nearest hundred
    }
}
