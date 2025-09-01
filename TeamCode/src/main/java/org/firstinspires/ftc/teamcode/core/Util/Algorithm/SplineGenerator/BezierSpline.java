package org.firstinspires.ftc.teamcode.core.Util.Algorithm.SplineGenerator;

import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.FollowerConstants.resolution;

import org.firstinspires.ftc.teamcode.core.Util.Math.Vector;
import org.firstinspires.ftc.teamcode.core.Util.utils.Constants;

import java.util.ArrayList;
import java.util.List;

public class BezierSpline implements Spline {

    private List<CubicBezierCurve> curves = new ArrayList<>();
    double targetAngle = Double.POSITIVE_INFINITY;
    double firstAngle = 0;
    private double length = 0;
    private double[][] dashboardDrawingPoints = new double[resolution + 1][resolution + 1];
    private final int searchStepLimit_BinarySearch = 10;
    private ArrayList<Double> lengthArray = new ArrayList<>();

    public BezierSpline() {
        this.curves = new ArrayList<>();
    }

    public BezierSpline(CubicBezierCurve... curves) {
        for (CubicBezierCurve curve : curves)
            addCurve(curve);
        computeLength();
    }

    public BezierSpline(double targetAngle, CubicBezierCurve... curves) {
        this.targetAngle = targetAngle;
        for (CubicBezierCurve curve : curves)
            addCurve(curve);
        computeLength();
    }

    public void addCurve(CubicBezierCurve curve) {
        this.curves.add(curve);
    }

    public Vector calculate(double t) {
        int segmentCount = curves.size();
        double segmentLength = 1.0 / segmentCount;
        int segmentIndex = Math.min((int) (t / segmentLength), segmentCount - 1);
        double localT = (t - segmentIndex * segmentLength) / segmentLength;

        return curves.get(segmentIndex).calculate(localT);
    }

    public Vector firstDerivative(double t) {
        int segmentCount = curves.size();
        double segmentLength = 1.0 / segmentCount;
        int segmentIndex = Math.min((int) (t / segmentLength), segmentCount - 1);
        double localT = (t - segmentIndex * segmentLength) / segmentLength;

        return curves.get(segmentIndex).firstDerivative(localT);
    }

    public Vector secondDerivative(double t) {
        int segmentCount = curves.size();
        double segmentLength = 1.0 / segmentCount;
        int segmentIndex = Math.min((int) (t / segmentLength), segmentCount - 1);
        double localT = (t - segmentIndex * segmentLength) / segmentLength;

        return curves.get(segmentIndex).secondDerivative(localT);
    }

    private int getSegmentIndex(double t) {
        int segmentCount = curves.size();
        double segmentLength = 1.0 / segmentCount;
        return Math.min((int) (t / segmentLength), segmentCount - 1);
    }

    public double slope(double t) {
        Vector dt = firstDerivative(t);
        return dt.getY() / dt.getX();
    }

    public double heading(double t) {
        int segmentCount = curves.size();
        double segmentLength = 1.0 / segmentCount;
        int segmentIndex = Math.min((int) (t / segmentLength), segmentCount - 1);
        targetAngle = curves.get(segmentIndex).getTargetAngle();
        if (targetAngle == Double.POSITIVE_INFINITY)
            return firstDerivative(t).getRelativeHeading() - firstDerivative(0).getRelativeHeading() + firstAngle;
        else
            return targetAngle;
    }

    public double curvatureOfThePath(double t) {
        Vector derivative = firstDerivative(t);
        Vector secondDerivative = secondDerivative(t);

        if (derivative.getMagnitude() == 0) return 0;
        return (Vector.crossProduct(derivative, secondDerivative)) / Math.pow(derivative.getMagnitude(), 3);
    }

    public Vector pathNormalVect(double t) {
        double curr = firstDerivative(t).getRelativeHeading();
        double deltaCurr = firstDerivative(t + 1.00 / resolution).getRelativeHeading();

        return Vector.polar(1, deltaCurr - curr);
    }

    @Override
    public double findClosestPoint(Vector point, double lastT) {
        double minT = -1;
        double minDist = Double.POSITIVE_INFINITY;
        for (double i = lastT * resolution; i <= resolution; i++) {
            double t = (i / (double) resolution);
            double dist = calculateMinimizationFunction(t, point);
            if (dist < minDist) {
                minDist = dist;
                minT = t;
            }
        }
        if (calculateMinimizationFunction(lastT - (double) 1 / resolution, point) <= minDist) {
            double N = (lastT - (double) 1 / resolution) * resolution;
            for (double i = 0; i <= N; i++) {
                double t = (i / (double) resolution);
                double dist = calculateMinimizationFunction(t, point);
                if (dist < minDist) {
                    minDist = dist;
                    minT = t;
                }
            }
        }
        if (minT == -1)
            minT = 1;
        return minT;
    }

    @Override
    public double findClosestPoint(Vector point) {
        double p = 0;
        double u = 1;
        int step = 0;
        while (step <= searchStepLimit_BinarySearch && p <= u) {
            double m = (p + u) / 2.0;
            double left = (p + m) / 2.0;
            double right = (u + m) / 2.0;
            if (calculateMinimizationFunction(left, point) <= calculateMinimizationFunction(right, point)) {
                u = m;
            } else {
                p = m;
            }
            step++;
        }
        return p;
    }


    @Override
    public double getLengthAt(double t) {
        int index = (int) (t * resolution);
        index = Math.min(resolution - 1, Math.max(0, index));

        // Calculate the fractional part of t * resolution for accurate interpolation
        double fractionalPart = t * resolution - index;

        // Interpolate between the length at index and index + 1
        double nextIndexLength = lengthArray.get(Math.min(resolution - 1, index + 1));
        double currentIndexLength = lengthArray.get(index);

        return currentIndexLength + (nextIndexLength - currentIndexLength) * fractionalPart;
    }

    private void computeLength() {
        double dt = 1.0 / (double) resolution;
        for (double d = 0; d <= 1; d += dt) {
            Vector currentPoint = calculate(d);
            dashboardDrawingPoints[1][(int) (d * resolution)] = -currentPoint.getX() / 2.54;
            dashboardDrawingPoints[0][(int) (d * resolution)] = currentPoint.getY() / 2.54;

            lengthArray.add(length);
            length += calculate(d).getMagnitude() * dt;
        }
    }

    @Override
    public double getLength() {
        return length;
    }

    @Override
    public void setFirstHeading(double angle) {
        this.firstAngle = angle;
    }

    @Override
    public boolean isClockWiseCurvature(double lastT, double currentT, Vector velocity) {
        Vector p1 = calculate(lastT);
        Vector p2 = calculate(currentT);
        Vector diff = p2.subtract(p1);
        return Vector.crossProduct(diff, velocity) < 0; // Clockwise if cross product is negative
    }


    private double calculateMinimizationFunction(double t, Vector pose) {
        return calculate(t).subtract(pose).getMagnitude();
    }

    @Override
    public double[][] getDashboardDrawingPoints() {
        return dashboardDrawingPoints;
    }
}
