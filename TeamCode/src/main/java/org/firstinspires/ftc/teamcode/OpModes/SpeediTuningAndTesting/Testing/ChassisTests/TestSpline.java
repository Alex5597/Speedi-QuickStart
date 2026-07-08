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

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        BezierSpline spline1 = new BezierSpline(
                new CubicBezierCurve(new Vector(0, 0), new Vector(0, 162), new Vector(65, 203), new Vector(60, 135)),
                new CubicBezierCurve(new Vector(60, 135), new Vector(55, 47), new Vector(111, -39), new Vector(102, 161))
        );

        SpeediDrive drive = new SpeediDrive(hardwareMap, new Pose(), telemetry, true);
        drive.setSpline_withTangentialHeadingChange(spline1);
        telemetry.addLine("GATA");

        waitForStart();

        boolean exitRequested = false;
        while (opModeIsActive() && !exitRequested) {
            while (opModeIsActive() && !drive.isDone()) {
                drive.update();
                if (gamepad1.a) exitRequested = true;
                telemetry.update();
            }
            if (exitRequested)
                break;
            drive.stop();

            drive.setTargetPose(new Pose(), true);
            while (opModeIsActive() && !drive.isDone()) {
                drive.update();
                if (gamepad1.a) exitRequested = true;
            }
            if (!exitRequested)
                drive.setSpline_withSlowerHeadingChange(spline1, 0.4);
        }
        drive.stop();
    }
}
