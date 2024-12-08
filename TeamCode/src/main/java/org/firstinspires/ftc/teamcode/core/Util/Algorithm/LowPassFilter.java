package org.firstinspires.ftc.teamcode.core.Util.Algorithm;

public class LowPassFilter {

    private final double t;
    private double lastValue;

    public LowPassFilter(double t, double initialValue) {
        this.t = t;
        this.lastValue = initialValue;
    }

    public double getValue(double rawValue) {
        if (Double.isNaN(rawValue)) {
            lastValue = 0;
            return 0;
        }
        double newValue = lastValue + t * (rawValue - lastValue);
        this.lastValue = newValue;
        return newValue;
    }

    public void resetFilter(double initialValue) {
        this.lastValue = initialValue;
    }
}