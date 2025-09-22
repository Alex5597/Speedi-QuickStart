package org.firstinspires.ftc.teamcode.OpModes.SpeediTuningAndTesting.Testing.ChassisTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Follower.SpeediDrive;
import org.firstinspires.ftc.teamcode.core.Util.Algorithm.SplineGenerator.CubicBezierCurve;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;
import org.firstinspires.ftc.teamcode.core.Util.Math.Vector;

@TeleOp
public class TestBezier extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        CubicBezierCurve curve = new CubicBezierCurve(new Vector(0, 0), new Vector(0, 80), new Vector(-60, 140), new Vector(100, 140));
        SpeediDrive drive = new SpeediDrive(hardwareMap, new Pose(), telemetry, true);
        waitForStart();
        drive.setSpline_withTangentialHeadingChange(curve);
        while (opModeIsActive()) {
            drive.update();
            drive.errorTelemetry(true);
            telemetry.update();
        }
    }
}
