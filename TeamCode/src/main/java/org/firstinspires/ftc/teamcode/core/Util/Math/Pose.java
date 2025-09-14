package org.firstinspires.ftc.teamcode.core.Util.Math;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.Objects;

public class Pose {
    private double x, y, heading;

    public Pose(double x, double y, DistanceUnit distanceUnit, double heading, AngleUnit angleUnit) {
        switch (distanceUnit) {
            case MM:
                this.x = x / 10;
                this.y = y / 10;
                break;
            case INCH:
                this.x = x * 2.54;
                this.y = y * 2.54;
                break;
            case METER:
                this.x = x * 100;
                this.y = y * 100;
                break;
            default:
                this.x = x;
                this.y = y;
                break;
        }
        switch (angleUnit) {
            case DEGREES:
                this.heading = angleWrapper(Math.toRadians(heading));
                break;
            default:
                this.heading = angleWrapper(heading);
                break;
        }
    }

    public Pose() {
        this(0, 0, DistanceUnit.CM, 0, AngleUnit.RADIANS);
    }

    public Pose(double x, double y, DistanceUnit distanceUnit) {
        this(x, y, distanceUnit, 0, AngleUnit.RADIANS);
    }

    public Pose(Vector vec, double heading) {
        this(vec.getX(), vec.getY(), DistanceUnit.CM, heading, AngleUnit.RADIANS);
    }

    public Pose(AprilTagPoseFtc ftcPose, DistanceUnit distanceUnit, AngleUnit angleUnit) {
        this(ftcPose.x * Math.cos(-ftcPose.yaw) - ftcPose.y * Math.sin(-ftcPose.yaw), ftcPose.x * Math.sin(-ftcPose.yaw) + ftcPose.y * Math.cos(-ftcPose.yaw), distanceUnit, -ftcPose.yaw, angleUnit);
    }

    public Pose(Pose2D pose) {
        this(pose.getX(DistanceUnit.CM), pose.getY(DistanceUnit.CM), DistanceUnit.CM, pose.getHeading(AngleUnit.RADIANS), AngleUnit.RADIANS);
    }

    public double getX(DistanceUnit distanceUnit) {
        switch (distanceUnit) {
            case MM:
                return x * 10;
            case INCH:
                return x / 2.54;
            case METER:
                return x / 100;
            default:
                return x;
        }
    }

    public void setX(double x, DistanceUnit distanceUnit) {
        switch (distanceUnit) {
            case MM:
                this.x = x / 10;
                break;
            case INCH:
                this.x = x * 2.54;
                break;
            case METER:
                this.x = x * 100;
                break;
            case CM:
                this.x = x;
                break;
        }
    }

    public double getY(DistanceUnit distanceUnit) {
        switch (distanceUnit) {
            case MM:
                return y * 10;
            case INCH:
                return y / 2.54;
            case METER:
                return y / 100;
            default:
                return y;
        }
    }

    public void setY(double y, DistanceUnit distanceUnit) {
        switch (distanceUnit) {
            case MM:
                this.y = y / 10;
                break;
            case INCH:
                this.y = y * 2.54;
                break;
            case METER:
                this.y = y * 100;
                break;
            case CM:
                this.y = y;
                break;
        }
    }

    public double getHeading(AngleUnit angleUnit) {
        if (Objects.requireNonNull(angleUnit) == AngleUnit.DEGREES) {
            return Math.toDegrees(heading);
        }
        return heading;
    }

    public void setHeading(double heading, AngleUnit angleUnit) {
        if (Objects.requireNonNull(angleUnit) == AngleUnit.DEGREES) {
            heading = Math.toRadians(heading);
        }
        this.heading = angleWrapper(heading);
    }

    public Pose add(Pose other) {
        return new Pose(x + other.x, y + other.y, DistanceUnit.CM, angleWrapper(heading + other.heading), AngleUnit.RADIANS);
    }

    public Pose subtract(Pose other) {
        return new Pose(x - other.x, y - other.y, DistanceUnit.CM, angleWrapper(heading - other.heading), AngleUnit.RADIANS);
    }

    public Pose multiplyBy(double scalar) {
        return new Pose(x * scalar, y * scalar, DistanceUnit.CM, angleWrapper(heading * scalar), AngleUnit.RADIANS);
    }

    public double distanceTo(Pose other, DistanceUnit distanceUnit) {
        if (other == null) return 0;
        return Math.hypot(other.getX(distanceUnit) - this.getX(distanceUnit), other.getY(distanceUnit) - this.getY(distanceUnit));
    }

    public String toString() {
        return "Pose(" + x + ", " + y + ", " + Math.toDegrees(heading) + ")";
    }

    public Vector toVec() {
        return new Vector(x, y, heading);
    }

    public Pose2D toPose2D() {
        return new Pose2D(DistanceUnit.CM, x, y, AngleUnit.RADIANS, heading);
    }

    public Pose invCoord() {
        return new Pose(y, x, DistanceUnit.CM, heading, AngleUnit.RADIANS);
    }

    public Pose rotateFieldCoordinate(double angle) {
        return new Pose(
                Math.cos(angle) * x + Math.sin(angle) * y,
                -Math.sin(angle) * x + Math.cos(angle) * y,
                DistanceUnit.CM,
                heading, AngleUnit.RADIANS);
    }

    public boolean isNaN() {
        return Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(heading);
    }

    private double angleWrapper(double angle) {
        angle %= (2.0 * Math.PI);
        if (angle > Math.PI) angle -= 2.0 * Math.PI;
        if (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }
}
