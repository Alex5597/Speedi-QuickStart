package org.firstinspires.ftc.teamcode.OpModes.SpeediTuningAndTesting.Testing.ChassisTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Follower.SpeediDrive;
import org.firstinspires.ftc.teamcode.core.Util.Algorithm.SplineGenerator.BezierSpline;
import org.firstinspires.ftc.teamcode.core.Util.Algorithm.SplineGenerator.CubicBezierCurve;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;
import org.firstinspires.ftc.teamcode.core.Util.Math.Vector;

@Config
@TeleOp(name = "Test LQR Spline")
public class TestLQRSpline extends LinearOpMode {
    public static double p0X = 0;
    public static double p0Y = 0;
    public static double p1X = 0;
    public static double p1Y = 162;
    public static double p2X = 65;
    public static double p2Y = 203;
    public static double p3X = 60;
    public static double p3Y = 135;
    public static double p4X = 55;
    public static double p4Y = 47;
    public static double p5X = 111;
    public static double p5Y = -39;
    public static double p6X = 102;
    public static double p6Y = 161;
    public static boolean useSecondCurve = true;
    public static boolean useFixedHeading = false;
    public static double fixedHeadingDegrees = 0;
    //the spline is defined in absolute coordinates starting at (0,0), so without resetting the pose
    //a restart makes the robot chase a target that begins back at the path start
    public static boolean resetPoseOnRestart = true;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        BezierSpline spline = buildSpline();
        SpeediDrive drive = new SpeediDrive(hardwareMap, new Pose(), telemetry, true);
        drive.setSpline_withLQR(spline);
        boolean wasA = false;
        boolean wasB = false;

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a && !wasA) {
                drive.stop();
                if (resetPoseOnRestart) drive.resetPosition(new Pose());
                spline = buildSpline();
                drive.setSpline_withLQR(spline);
            }
            wasA = gamepad1.a;

            if (gamepad1.b && !wasB) {
                drive.setRunMode(SpeediDrive.RunMode.MANUAL);
                drive.stop();
                break;
            }
            wasB = gamepad1.b;

            drive.update();
            if (drive.lqrFollower != null) drive.lqrFollower.telemetry(true);
            drive.currentPosTelemetry(true);
            telemetry.addData("Done", drive.isDone());
            telemetry.addData("Progress", drive.getPercentageOfTrajectoryDone());
            telemetry.update();
        }
    }

    private BezierSpline buildSpline() {
        CubicBezierCurve first = new CubicBezierCurve(
                new Vector(p0X, p0Y),
                new Vector(p1X, p1Y),
                new Vector(p2X, p2Y),
                new Vector(p3X, p3Y)
        );
        if (!useSecondCurve) {
            if (useFixedHeading) return new BezierSpline(Math.toRadians(fixedHeadingDegrees), first);
            return new BezierSpline(first);
        }

        CubicBezierCurve second = new CubicBezierCurve(
                new Vector(p3X, p3Y),
                new Vector(p4X, p4Y),
                new Vector(p5X, p5Y),
                new Vector(p6X, p6Y)
        );
        if (useFixedHeading) return new BezierSpline(Math.toRadians(fixedHeadingDegrees), first, second);
        return new BezierSpline(first, second);
    }
}
