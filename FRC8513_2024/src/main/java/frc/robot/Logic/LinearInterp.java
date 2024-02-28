package frc.robot.Logic;

import java.util.ArrayList;

public class LinearInterp {

    private ArrayList<Double> xVals = new ArrayList<Double>();
    private ArrayList<Double> yVals = new ArrayList<Double>();

    // assume this takes in sorted by x
    public LinearInterp(double[] x, double[] y) {
        for (int i = 0; i < x.length; i++) {
            xVals.add(x[i]);
            yVals.add(y[i]);
        }

    }

    public double interpolateLinearly(double x) {
        int startX = -1;
        if (x > xVals.get(xVals.size() - 1) || x < xVals.get(0)) {
            return -1;
        }
        for (int i = 0; xVals.get(i).doubleValue() < x; i++) {
            startX = i;
        }
        double deltaX = xVals.get(startX + 1) - xVals.get(startX);
        double deltaY = yVals.get(startX + 1) - yVals.get(startX);

        double percentageAlongWay = (x - xVals.get(startX)) / deltaX;
        return percentageAlongWay * deltaY + yVals.get(startX);

    }

}
