package org.firstinspires.ftc.teamcode.OpModes.SpeediTuningAndTesting.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.core.Util.Algorithm.SplineGenerator.BezierSpline;
import org.firstinspires.ftc.teamcode.core.Util.Algorithm.SplineGenerator.CubicBezierCurve;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;
import org.firstinspires.ftc.teamcode.core.Util.Math.Vector;

@TeleOp
public class CentripetalForceTuner extends LinearOpMode {

    int state = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        BezierSpline spline1 = new BezierSpline(new CubicBezierCurve(new Vector(0, 0), new Vector(-150, 0), new Vector(-150, 150), new Vector(0, 150), 0), new CubicBezierCurve(new Vector(0, 150), new Vector(150, 150), new Vector(150, 0), new Vector(5, 0), 0));

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose(), telemetry, true);
        drive.setSpline_withInstantHeadingChange(spline1);
        telemetry.addLine("GATA");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            while (opModeIsActive() && !drive.isDone()) {
                drive.update();
                telemetry.update();
            }
            drive.setTargetPose(new Pose(),true);
            while (opModeIsActive() && !drive.isDone()) {
                drive.update();
                telemetry.update();
            }
            drive.setSpline_withInstantHeadingChange(spline1);
        }
    }
}
