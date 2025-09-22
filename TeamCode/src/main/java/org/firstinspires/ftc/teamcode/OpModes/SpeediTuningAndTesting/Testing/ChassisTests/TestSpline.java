package org.firstinspires.ftc.teamcode.OpModes.SpeediTuningAndTesting.Testing.ChassisTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Follower.SpeediDrive;
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
        BezierSpline spline1 = new BezierSpline(
                new CubicBezierCurve(new Vector(0, 0), new Vector(0, 162), new Vector(65, 203), new Vector(60, 135)),
                new CubicBezierCurve(new Vector(60, 135), new Vector(55, 47), new Vector(111, -39), new Vector(102, 161))
        );

        BezierSpline spline2 = new BezierSpline(new CubicBezierCurve(new Vector(30, 150), new Vector(-30, 132.7), new Vector(-46, -120), new Vector(-46, 15), Math.toRadians(-35)));

//        for (double i = 0; i <= 1; i += 10.0 / resolution) {
//            telemetry.addData("First derivative AT t=" + i + " is", new Pose(spline1.calculate(i), spline1.heading(i)).toString());
//            telemetry.addData("POSE AT t=" + i + " is", new Pose(spline1.firstDerivative(i), spline1.heading(i)).toString());
//        }

        // DrawRobot.drawPath(spline1, "#3F51B5");

        SpeediDrive drive = new SpeediDrive(hardwareMap, new Pose(), telemetry, true);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        drive.setSpline_withTangentialHeadingChange(spline1);
        telemetry.addLine("GATA");
//        telemetry.addLine(drive.getTarget().toString());
//        telemetry.update();


        waitForStart();


        while (opModeIsActive()) {
            while (opModeIsActive() && !drive.isDone()) {
                drive.update();
                telemetry.update();
            }
            if (gamepad1.a)
                break;
            drive.stop();

            drive.setTargetPose(new Pose(), true);
            while (opModeIsActive() && !drive.isDone()) {
                drive.update();
            }
            drive.setSpline_withSlowerHeadingChange(spline1, 0.4);
        }
    }
}
