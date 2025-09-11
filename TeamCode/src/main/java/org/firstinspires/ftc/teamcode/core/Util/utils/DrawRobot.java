package org.firstinspires.ftc.teamcode.core.Util.utils;

import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.WAIT_TIME_VARIABLE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.core.Util.Algorithm.SplineGenerator.Spline;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;
import org.firstinspires.ftc.teamcode.core.Util.Math.Vector;


public class DrawRobot {
    private static TelemetryPacket packet;

    public static void drawDebug(MecanumDrive drive) {
        Pose pose = drive.getLocalizerInstance().getPoseEstimate();
        if (drive.getRunMode() == MecanumDrive.RunMode.Spline) {
            Spline spline = drive.curve;
            drawPath(spline, "#3F51B5");
        } else if (drive.getRunMode() == MecanumDrive.RunMode.PID && drive.getTarget().getX(DistanceUnit.CM) != WAIT_TIME_VARIABLE) {
            drawGoToPoint(pose, drive.getTarget(), "#3F51B5");
        }
        drawPoseHistory(drive.poseTracker, "#4CAF50");
        if (drive.noGoZone)
            drawNoGoZone(drive.topLeftCorner, drive.topRightCorner, drive.bottomLeftCorner, drive.bottomRightCorner);
        drawRobot(pose, "#4CAF50");
        sendPacket();
    }

    private static void drawGoToPoint(Pose current, Pose target, String color) {
        if (packet == null) packet = new TelemetryPacket();
        double[][] points = new double[2][2];
        points[0][0] = current.getY(DistanceUnit.INCH);
        points[0][1] = target.getY(DistanceUnit.INCH);

        points[1][0] = -current.getX(DistanceUnit.INCH);
        points[1][1] = -target.getX(DistanceUnit.INCH);

        packet.fieldOverlay().setStroke(color);
        drawPath(packet.fieldOverlay(), points);
    }

    public static void drawRobot(Pose pose) {
        drawRobot(pose, "#4CAF50");
    }

    public static void drawRobot(Pose pose, String color) {
        if (packet == null) packet = new TelemetryPacket();

        packet.fieldOverlay().setStroke(color);
        DrawRobot.drawRobotOnCanvas(packet.fieldOverlay(), pose);
    }

    public static void drawPath(Spline path, String color) {
        if (packet == null) packet = new TelemetryPacket();

        packet.fieldOverlay().setStroke(color);
        DrawRobot.drawPath(packet.fieldOverlay(), path.getDashboardDrawingPoints());
    }


    private static void drawPoseHistory(DashboardPoseTracker poseTracker, String color) {
        if (packet == null) packet = new TelemetryPacket();

        packet.fieldOverlay().setStroke(color);
        packet.fieldOverlay().strokePolyline(poseTracker.getXPositionsArray(), poseTracker.getYPositionsArray());
    }

    public static void sendPacket() {
        if (packet != null) {
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            packet = null;
        }
    }

    private static void drawRobotOnCanvas(Canvas c, Pose t) {
        t = t.invCoord();
        t.setY(-t.getY(DistanceUnit.INCH), DistanceUnit.INCH);
        final double ROBOT_RADIUS = 9;

        c.setStrokeWidth(1);
        c.strokeCircle(t.getX(DistanceUnit.INCH), t.getY(DistanceUnit.INCH), ROBOT_RADIUS);

        Vector halfv = new Vector(Math.cos(t.getHeading(AngleUnit.RADIANS)), Math.sin(t.getHeading(AngleUnit.RADIANS))).scalarMultiply(ROBOT_RADIUS * 0.5);
        Vector p1 = new Vector(t.getX(DistanceUnit.INCH) + halfv.getX(), t.getY(DistanceUnit.INCH) + halfv.getY());
        Vector p2 = p1.add(halfv);
        c.strokeLine(p1.getX(), p1.getY(), p2.getX(), p2.getY());
    }

    public static void drawNoGoZone(Pose topLeftCorner, Pose topRightCorner, Pose bottomLeftCorner, Pose bottomRightCorner) {
        if (packet == null) packet = new TelemetryPacket();
        Canvas c = packet.fieldOverlay();
        c.setStroke("red");
        c.setStrokeWidth(1);
        c.strokePolygon(new double[]{topLeftCorner.getY(DistanceUnit.INCH), topRightCorner.getY(DistanceUnit.INCH), bottomRightCorner.getY(DistanceUnit.INCH), bottomLeftCorner.getY(DistanceUnit.INCH)}, new double[]{-topLeftCorner.getX(DistanceUnit.INCH), -topRightCorner.getX(DistanceUnit.INCH), -bottomRightCorner.getX(DistanceUnit.INCH), -bottomLeftCorner.getX(DistanceUnit.INCH)});
    }

    private static void drawPath(Canvas c, double[][] points) {
        c.strokePolyline(points[0], points[1]);
    }
}