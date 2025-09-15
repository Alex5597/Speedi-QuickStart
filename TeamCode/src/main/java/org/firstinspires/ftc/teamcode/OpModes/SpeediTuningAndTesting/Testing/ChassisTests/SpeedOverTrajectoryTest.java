package org.firstinspires.ftc.teamcode.OpModes.SpeediTuningAndTesting.Testing.ChassisTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Follower.SplineFollower;
import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Localizer.Localizer;
import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Localizer.PinPointLocalizer;
import org.firstinspires.ftc.teamcode.core.Util.Algorithm.SplineGenerator.BezierSpline;
import org.firstinspires.ftc.teamcode.core.Util.Algorithm.SplineGenerator.CubicBezierCurve;
import org.firstinspires.ftc.teamcode.core.Util.Algorithm.SplineGenerator.Spline;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;
import org.firstinspires.ftc.teamcode.core.Util.Math.Vector;

@TeleOp
public class SpeedOverTrajectoryTest extends LinearOpMode {
    SplineFollower follower;
    Localizer localizer;
    Spline spline;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        spline = new BezierSpline(new CubicBezierCurve(new Vector(0, 0), new Vector(-137, -50), new Vector(-43, 23), new Vector(-169, 23), Math.toRadians(0)));
        localizer = new PinPointLocalizer(hardwareMap, new Pose());
        follower = new SplineFollower(localizer.getPoseEstimate(), spline, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            localizer.update();
            //for (double i = 0; i <= 1; i += 0.001)
            // telemetry.addData("Viteza ideala la t= " + i + "este", follower.speedAtT(i));
            telemetry.addData("Viteza motor", follower.getMotorPower(localizer.getPoseEstimate(), localizer.getVelocity(), localizer.getGlideVector()).toString());
            telemetry.update();
        }
    }
}
