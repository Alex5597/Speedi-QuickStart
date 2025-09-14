package org.firstinspires.ftc.teamcode.core.Util.Math;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Vector {
    double magnitude;
    private double x, y, heading;

    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
        this.heading = 0;
        magnitude = Math.hypot(x, y);
    }

    public Vector(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        magnitude = Math.hypot(x, y);
    }

    public Vector(Pose2D vec) {
        this.x = vec.getX(CM);
        this.y = vec.getY(CM);
        this.heading = vec.getHeading(RADIANS);
        magnitude = Math.hypot(x, y);
    }

    public Vector(Pose vec) {
        this.x = vec.getX(CM);
        this.y = vec.getY(CM);
        this.heading = vec.getHeading(RADIANS);
        magnitude = Math.hypot(x, y);
    }
    public static Vector polar(double r, double t) {
        if (r < 0) {
            r = -r;
            t = normalizeAngle(t + Math.PI);
        } else {
            t = normalizeAngle(t);
        }
        return new Vector(r * Math.cos(t), r * Math.sin(t));
    }

    public static double dot(Vector a, Vector b) {
        return a.x * b.x + a.y * b.y;
    }

    public static double crossProduct(Vector a, Vector b) {
        return a.x * b.y - a.y * b.x;
    }

    public static double findScaleFactor(Vector staticVector, Vector variableVector) {
        double a = Math.pow(variableVector.getX(), 2) + Math.pow(variableVector.getY(), 2);
        double b = staticVector.getX() * variableVector.getX() + staticVector.getY() * variableVector.getY();
        double c = Math.pow(staticVector.getX(), 2) + Math.pow(staticVector.getY(), 2) - 1.0;
        return (-b + Math.sqrt(Math.pow(b, 2) - a * c)) / (a);
    }

    public static Vector add(Vector a, Vector b) {
        return new Vector(a.x + b.x, a.y + b.y, a.heading + b.heading);
    }

    public static Vector subtract(Vector a, Vector b) {
        return new Vector(a.x - b.x, a.y - b.y, a.heading - b.heading);
    }

    public static Vector scalarMultiply(Vector vec, double scalar) {
        return new Vector(vec.x * scalar, vec.y * scalar, vec.heading * scalar);
    }

    public static Vector scalarDivide(Vector vec, double scalar) {
        return new Vector(vec.x / scalar, vec.y / scalar, vec.heading / scalar);
    }

    public static Vector slerp(Vector a, Vector b, double t) {
        double aMag = a.getMagnitude();
        double aHead = a.getRelativeHeading();
        double bMag = b.getMagnitude();
        double bHead = b.getRelativeHeading();
        return polar((1 - t) * aMag + t * bMag, (1 - t) * aHead + t * bHead);
    }

    private static double angleWrapper(double angle) {
        if (angle > Math.PI) angle -= 2.0 * Math.PI;
        if (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    public static double normalizeAngle(double angleRadians) {
        double angle = angleRadians % (2 * Math.PI);
        if (angle < 0) {
            return angle + 2 * Math.PI;
        }
        return angle;
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
        magnitude = Math.hypot(x, y);
    }

    public double getY() {
        return y;
    }

    // Operations

    public void setY(double y) {
        this.y = y;
        magnitude = Math.hypot(x, y);
    }

    public double getHeading() {
        return heading;
    }

    public void setHeading(double heading) {
        this.heading = heading;
    }

    public double getRelativeHeading() {
        return Math.atan2(y, x);
    }

    public double getMagnitude() {
        return magnitude;
    }

    public double getMagnitude_AngularAsWell() {
        return Math.sqrt(x * x + y * y + heading * heading);
    }

    public double getMagSq() {
        return magnitude * magnitude;
    }

    public Vector add(Vector other) {
        return add(this, other);
    }

    public Vector subtract(Vector other) {
        return subtract(this, other);
    }

    public Vector scalarMultiply(double scalar) {
        return scalarMultiply(this, scalar);
    }

    public Vector scalarDivide(double scalar) {
        return scalarDivide(this, scalar);
    }

    public Vector rotate(double angle) {
        return new Vector(Math.cos(angle) * x + Math.sin(angle) * y, Math.cos(angle) * y - Math.sin(angle) * x, heading);
    }

    public Vector scaleToMagnitude(double targetMagnitude) {
        double currentMagnitude = getMagnitude();
        if (currentMagnitude <= 1) return this;
        scaleBy(1.0 / currentMagnitude);
        scaleBy(targetMagnitude);
        return this;
    }

    public Vector scaleToMagnitude_AngularAsWell(double targetMagnitude) {
        double currentMagnitude = getMagnitude_AngularAsWell();
        if (currentMagnitude <= 1) return this;
        scaleBy_AngularAsWell(1.00 / currentMagnitude);
        scaleBy_AngularAsWell(targetMagnitude);
        return this;
    }

    public void scaleBy(double a) {
        x = x * a;
        y = y * a;
        magnitude = Math.hypot(x, y);
    }

    public void scaleBy_AngularAsWell(double a) {
        x = x * a;
        y = y * a;
        heading = heading * a;
        magnitude = Math.hypot(x, y);
    }

    public void multiplyYby(double value) {
        y = y * value;
        magnitude = Math.hypot(x, y);
    }

    public void multiplyXby(double value) {
        x = x * value;
        magnitude = Math.hypot(x, y);
    }

    public String toString() {
        return "\n(" + x + "; " + y + "; " + heading + ")";
    }

    public Pose toPose() {
        return new Pose(x, y, CM, heading, RADIANS);
    }

    public Vector invCoord() {
        return new Vector(y, x, heading);
    }

    public boolean equals(Vector vector) {
        return vector.getX() == x && vector.getY() == y && vector.getHeading() == heading;
    }

    public boolean isNaN() {
        return Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(heading);
    }
}
