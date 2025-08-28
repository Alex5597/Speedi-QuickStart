package org.firstinspires.ftc.teamcode.CodeBTC.Core.Algorithms;

public class LowPassFilter {

    private final double tradeOff;
    private double lastValue;

    public LowPassFilter(double tradeOff, double initialValue) {
        this.tradeOff = tradeOff;
        this.lastValue = initialValue;
    }

    public double getValue(double rawValue) {
        if (Double.isNaN(rawValue)) {
            lastValue = 0;
            return 0;
        }
        double newValue = tradeOff * lastValue + (1 - tradeOff) * rawValue;
        this.lastValue = newValue;
        return newValue;
    }

    public void resetFilter(double initialValue) {
        this.lastValue = initialValue;
    }
}