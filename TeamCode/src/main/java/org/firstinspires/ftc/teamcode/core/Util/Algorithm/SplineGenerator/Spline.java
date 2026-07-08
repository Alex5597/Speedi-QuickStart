package org.firstinspires.ftc.teamcode.core.Util.Algorithm.SplineGenerator;

import org.firstinspires.ftc.teamcode.core.Util.Math.Vector;

public interface Spline {
    Vector calculate(double t);

    Vector firstDerivative(double t);

    Vector secondDerivative(double t);

    double curvatureOfThePath(double t);

    Vector pathNormalVect(double t);


    double heading(double t);

    double findClosestPoint(Vector position, double lastT);
    double findClosestPoint(Vector position);

    void setFirstHeading(double angle);
    void setTargetHeading(double angle);
    double getTargetAngle();

    boolean isClockWiseCurvature(double lastT, double currenT, Vector velocity);

    double getLengthAt(double t);

    double getTAtLength(double length);

    double getLength();
    double[][] getDashboardDrawingPoints();

}
