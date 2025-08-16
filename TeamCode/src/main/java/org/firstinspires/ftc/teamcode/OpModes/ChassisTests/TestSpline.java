package org.firstinspires.ftc.teamcode.OpModes.ChassisTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.core.Util.Algorithm.SplineGenerator.BezierSpline;
import org.firstinspires.ftc.teamcode.core.Util.Algorithm.SplineGenerator.CubicBezierCurve;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;
import org.firstinspires.ftc.teamcode.core.Util.Math.Vector;

@TeleOp
public class TestSpline extends LinearOpMode {

    int state = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        BezierSpline spline1 = new BezierSpline(new CubicBezierCurve(new Vector(0, 0), new Vector(-137, -50), new Vector(-43, 23), new Vector(-169, 23), Math.toRadians(0)));

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose(0, 0, Math.toRadians(0)), telemetry, true);
        drive.setSpline_withInstantHeadingChange(spline1);
        telemetry.addLine("GATA");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            while (opModeIsActive() && !drive.isDone()) {
                drive.update();
            }
            if (gamepad1.a)
                break;
            drive.setTargetPose(new Pose(0, 0, Math.toRadians(0)));
            while (opModeIsActive() && !drive.isDone()) {
                drive.update();
            }
            drive.setSpline_withInstantHeadingChange(spline1);
        }
    }
}
