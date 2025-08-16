package org.firstinspires.ftc.teamcode.core.Util.Algorithm.SplineGenerator;

import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.FollowerConstants.resolution;

import org.firstinspires.ftc.teamcode.core.Util.Math.Vector;

import java.util.ArrayList;

public class CubicBezierCurve implements Spline {

    private Vector p0, p1, p2, p3;
    double targetAngle = Double.POSITIVE_INFINITY;
    double firstAngle = 0;
    private double length = 0;
    private final int searchStepLimit_BinarySearch = 10;
    private double[][] dashboardDrawingPoints = new double[resolution + 1][resolution + 1];
    private ArrayList<Double> lengthArray = new ArrayList<>();

    public CubicBezierCurve(Vector p0, Vector p1, Vector p2, Vector p3) {
        this.p0 = p0;
        this.p1 = p1;
        this.p2 = p2;
        this.p3 = p3;
        computeLength();
    }

    public CubicBezierCurve(Vector p0, Vector p1, Vector p2, Vector p3, double targetAngle) {
        this.targetAngle = targetAngle;
        this.p0 = p0;
        this.p1 = p1;
        this.p2 = p2;
        this.p3 = p3;
        computeLength();
    }

    public Vector getP0() {
        return p0;
    }

    public void setP0(Vector p0) {
        this.p0 = p0;
    }

    public Vector getP1() {
        return p1;
    }

    public void setP1(Vector p1) {
        this.p1 = p1;
    }

    public Vector getP2() {
        return p2;
    }

    public void setP2(Vector p2) {
        this.p2 = p2;
    }

    public Vector getP3() {
        return p3;
    }

    public void setP3(Vector p3) {
        this.p3 = p3;
    }

    public Vector calculate(double t) {
        // (1 - t)^3 * P0 + 3 * t * (1 - t)^2 * P1 + 3 * t^2 * (1 - t) * P2 + t^3 * P3
        double w = 1 - t;
        Vector firstTerm = p0.scalarMultiply(w * w * w);
        Vector secondTerm = p1.scalarMultiply(3 * t * w * w);
        Vector thirdTerm = p2.scalarMultiply(3 * t * t * w);
        Vector fourthTerm = p3.scalarMultiply(t * t * t);
        return firstTerm.add(secondTerm).add(thirdTerm).add(fourthTerm);
    }

    public Vector firstDerivative(double t) {
        double w = 1 - t;
        Vector firstTerm = p1.subtract(p0).scalarMultiply(3 * w * w);
        Vector secondTerm = p2.subtract(p1).scalarMultiply(6 * w * t);
        Vector thirdTerm = p3.subtract(p2).scalarMultiply(3 * t * t);
        return firstTerm.add(secondTerm).add(thirdTerm);
    }

    public Vector secondDerivative(double t) {
        double w = 1 - t;
        Vector doubP2 = p2.scalarMultiply(2);
        Vector doubP1 = p1.scalarMultiply(2);

        Vector firstTerm = p2.subtract(doubP1).add(p0).scalarMultiply(6 * w);
        Vector secondTerm = p3.subtract(doubP2).add(p1).scalarMultiply(6 * t);
        return firstTerm.add(secondTerm);
    }

    public double slope(double t) {
        Vector dt = firstDerivative(t);
        return dt.getY() / dt.getX();
    }

    public double heading(double t) {
        if (targetAngle == Double.POSITIVE_INFINITY)
            return firstDerivative(t).getRelativeHeading() - firstAngle;
        else
            return targetAngle;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double curvatureOfThePath(double t) {
        Vector derivative = firstDerivative(t);
        Vector secondDerivative = secondDerivative(t);

        if (derivative.getMagnitude() == 0) return 0;
        return (Vector.crossProduct(derivative, secondDerivative)) / Math.pow(derivative.getMagnitude(), 3);
    }

    public Vector circumcenter(Vector p1, Vector p2, Vector p3) {   //Calculates the circumcenter of the triangle defined by the last three points p1, p2, p3
        double ax = p1.getX();
        double ay = p1.getY();
        double bx = p2.getX();
        double by = p2.getY();
        double cx = p3.getX();
        double cy = p3.getY();
        double d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
        double ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / d;
        double uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / d;
        return new Vector(ux, uy);
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
        double lower = 0;
        double upper = 1;

        for (int i = 0; i < searchStepLimit_BinarySearch; i++) {
            if (calculateMinimizationFunction(lower + 0.25 * (upper - lower), point) >
                    calculateMinimizationFunction(lower + 0.75 * (upper - lower), point)) {
                lower += (upper - lower) / 2.0;
            } else {
                upper -= (upper - lower) / 2.0;
            }
        }

        return lower + 0.5 * (upper - lower);
    }

    @Override
    public boolean isClockWiseCurvature(double lastT, double currentT, Vector velocity) {
        Vector p1 = calculate(lastT);
        Vector p2 = calculate(currentT);
        Vector diff = p2.subtract(p1);
        return Vector.crossProduct(diff, velocity) < 0; // Clockwise if cross product is negative
    }

    @Override
    public void setFirstHeading(double angle) {
        this.firstAngle = angle;
    }

    private double calculateMinimizationFunction(double t, Vector point) {
        return calculate(t).subtract(point).getMagnitude();
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
    public double[][] getDashboardDrawingPoints() {
        return dashboardDrawingPoints;
    }
}