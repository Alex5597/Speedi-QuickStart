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
        CubicBezierCurve curve = new CubicBezierCurve(new Vector(0, 0), new Vector(14, -172), new Vector(-77, 83), new Vector(-70, -200), 0);
        SpeediDrive drive = new SpeediDrive(hardwareMap, new Pose(), telemetry, true);
        drive.setSpline_withInstantHeadingChange(curve);

        waitForStart();

        while (!drive.isDone()) {
            drive.update();
            //drive.errorTelemetry(false);
            telemetry.update();
        }
    }
}
