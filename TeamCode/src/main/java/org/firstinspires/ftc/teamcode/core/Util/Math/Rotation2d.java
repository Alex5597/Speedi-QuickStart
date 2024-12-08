package org.firstinspires.ftc.teamcode.core.Util.Math;

public class Rotation2d {
    public double real;
    public double imag;

    public Rotation2d(double real, double imag) {
        this.real = real;
        this.imag = imag;
    }

    public static Rotation2d exp(double theta) {
        return new Rotation2d(Math.cos(theta), Math.sin(theta));
    }

    public static Rotation2d fromDouble(double theta) {
        return exp(theta);
    }

    public Rotation2d plus(double x) {
        return times(exp(x));
    }

    public Rotation2d minus(Rotation2d r) {
        return r.inverse().times(this);
    }

    public Vector times(Vector v) {
        return new Vector(real * v.getX() - imag * v.getY(), imag * v.getX() + real * v.getY());
    }

    public Rotation2d times(Rotation2d r) {
        return new Rotation2d(real * r.real - imag * r.imag, real * r.imag + imag * r.real);
    }

    public Vector vec() {
        return new Vector(real, imag);
    }

    public Rotation2d inverse() {
        return new Rotation2d(real, -imag);
    }

    public double log() {
        return Math.atan2(imag, real);
    }

    public double toDouble() {
        return log();
    }
}
