package org.firstinspires.ftc.teamcode.core.Util.Math;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

public class Pose {
    private double x, y, heading;

    public Pose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose() {
        this(0, 0, 0);
    }

    public Pose(double x, double y) {
        this(x, y, 0);
    }

    public Pose(Vector vec, double heading) {
        this(vec.getX(), vec.getY(), heading);
    }

    public Pose(AprilTagPoseFtc ftcPose) {
        this.heading = Math.toRadians(-ftcPose.yaw);
        this.x = (ftcPose.x * Math.cos(heading) - ftcPose.y * Math.sin(heading)) * 2.54;
        this.y = (ftcPose.x * Math.sin(heading) + ftcPose.y * Math.cos(heading)) * 2.54;
    }

    public Pose(Pose2D pose) {
        this(pose.getX(DistanceUnit.CM), pose.getY(DistanceUnit.CM), pose.getHeading(AngleUnit.RADIANS));
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public void setHeading(double heading) {
        this.heading = heading;
    }

    public Pose add(Pose other) {
        return new Pose(x + other.x, y + other.y, heading + other.heading);
    }

    public Pose subtract(Pose other) {
        return new Pose(x - other.x, y - other.y, heading - other.heading);
    }

    public Pose multiplyBy(double scalar) {
        return new Pose(x * scalar, y * scalar, heading * scalar);
    }

    public String toString() {
        return "Pose(" + x + "\n" + y + "\n" + Math.toDegrees(heading) + ")";
    }

    public Pose toInches() {
        return new Pose(x / 2.54, y / 2.54, heading);
    }

    public Vector toVec() {
        return new Vector(x, y, heading);
    }

    public Pose2D toPose2D() {
        return new Pose2D(DistanceUnit.CM, x, y, AngleUnit.RADIANS, heading);
    }

    public Pose invCoord() {
        return new Pose(y, x, heading);
    }

    public Pose rotateFieldCoordinate(double angle) {
        return new Pose(
                Math.cos(angle) * x + Math.sin(angle) * y,
                Math.cos(angle) * y - Math.sin(angle) * x,
                heading
        );
    }

    public boolean isNaN() {
        return Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(heading);
    }
}
