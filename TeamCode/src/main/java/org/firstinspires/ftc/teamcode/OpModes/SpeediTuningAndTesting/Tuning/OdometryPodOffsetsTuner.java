package org.firstinspires.ftc.teamcode.OpModes.SpeediTuningAndTesting.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Follower.SpeediDrive;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;
import org.firstinspires.ftc.teamcode.core.Util.Math.Vector;

/**
 * Spins the robot in place and calculates the two Pinpoint pod offsets, exactly as
 * GoBildaPinpointDriver.setOffsets expects them:
 * <ul>
 * <li><b>xPodOffsetInMM</b> - how far SIDEWAYS the X (forward) pod is from the center of rotation, LEFT positive</li>
 * <li><b>yPodOffsetInMM</b> - how far FORWARD the Y (strafe) pod is from the center of rotation, FORWARD positive</li>
 * </ul>
 * Copy both results into Constants.LocalizerConstants.
 * <p>
 * How it works: when the robot only rotates (CCW positive heading), a pod mounted off-center still
 * slides, so its ticks change proportionally to its offset:
 * X pod: dTicksX/dHeading = -xPodOffset, Y pod: dTicksY/dHeading = +yPodOffset (in mm per radian).
 * A least squares fit of ticks vs heading gives the slopes, and the offsets follow directly.
 * <p>
 * DO STEP 2 OF THE README FIRST: driving forward must increase the X ticks and strafing LEFT must
 * increase the Y ticks (Pinpoint convention), otherwise the signs come out flipped.
 */
@Config
@TeleOp(name = "Odometry Pod Offsets Tuner")
public class OdometryPodOffsetsTuner extends LinearOpMode {
    public static boolean autoSpin = true;//set false to rotate the robot by hand
    public static double rotatePower = 0.25;
    public static double targetRotationDegrees = 1080;
    //mm travelled per encoder tick: goBILDA 4-bar pod = 1/19.89436789, swingarm pod = 1/13.26291192
    public static double mmPerTickX = 1.0 / 19.89436789;
    public static double mmPerTickY = 1.0 / 19.89436789;

    private SpeediDrive drive;

    //least squares sums of pod distance (mm) vs heading (rad)
    private int samples;
    private double sumT, sumTT, sumX, sumTX, sumY, sumTY;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SpeediDrive(hardwareMap, new Pose(), telemetry, true);
        drive.setRunMode(SpeediDrive.RunMode.MANUAL);

        telemetry.addLine("The robot will rotate " + targetRotationDegrees + " degrees in place, keep the area clear");
        telemetry.addLine("(README step 2 must already be done: forward -> X ticks up, strafe LEFT -> Y ticks up)");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        double lastHeading = heading();
        double unwrappedHeading = 0;
        double startXTicks = drive.localizer.odo.getEncoderX();
        double startYTicks = drive.localizer.odo.getEncoderY();

        while (opModeIsActive() && Math.abs(Math.toDegrees(unwrappedHeading)) < targetRotationDegrees) {
            //left side backwards + right side forwards = counter-clockwise
            if (autoSpin) drive.motors.setMotorPower(-rotatePower, -rotatePower, rotatePower, rotatePower);
            drive.update();

            double now = heading();
            unwrappedHeading += Vector.wrapAngle(now - lastHeading);
            lastHeading = now;

            double xPodMm = (drive.localizer.odo.getEncoderX() - startXTicks) * mmPerTickX;
            double yPodMm = (drive.localizer.odo.getEncoderY() - startYTicks) * mmPerTickY;
            addSample(unwrappedHeading, xPodMm, yPodMm);

            telemetry.addData("Rotation (deg)", Math.toDegrees(unwrappedHeading));
            telemetry.addData("X pod distance (mm)", xPodMm);
            telemetry.addData("Y pod distance (mm)", yPodMm);
            telemetry.addData("Samples", samples);
            telemetry.update();
        }
        drive.motors.setMotorPower(0, 0, 0, 0);

        //xOffset is sideways-left of the X pod: its ticks change at -offset per radian of CCW rotation
        double xPodOffsetInMM = -slope(sumX, sumTX);
        //yOffset is forward of the Y pod: its ticks change at +offset per radian of CCW rotation
        double yPodOffsetInMM = slope(sumY, sumTY);

        while (opModeIsActive()) {
            telemetry.addLine("====== RESULTS: copy into Constants.LocalizerConstants ======");
            telemetry.addData("xPodOffsetInMM (X/forward pod, LEFT positive)", xPodOffsetInMM);
            telemetry.addData("yPodOffsetInMM (Y/strafe pod, FORWARD positive)", yPodOffsetInMM);
            telemetry.addData("Samples used", samples);
            telemetry.addLine("Sanity check the signs with a ruler: pod left of center -> x positive, pod in front of center -> y positive");
            telemetry.update();
            sleep(100);
        }
    }

    private double heading() {
        //SPEEDI heading equals the Pinpoint heading (rotateFieldCoordinate only rotates x/y), CCW positive
        return drive.localizer.getPoseEstimate().getHeading(AngleUnit.RADIANS);
    }

    private void addSample(double headingRad, double xPodMm, double yPodMm) {
        samples++;
        sumT += headingRad;
        sumTT += headingRad * headingRad;
        sumX += xPodMm;
        sumTX += headingRad * xPodMm;
        sumY += yPodMm;
        sumTY += headingRad * yPodMm;
    }

    /**
     * Least squares slope of pod distance vs heading, in mm per radian
     */
    private double slope(double sumValue, double sumHeadingValue) {
        double denominator = samples * sumTT - sumT * sumT;
        if (Math.abs(denominator) <= 1e-9) return 0;
        return (samples * sumHeadingValue - sumT * sumValue) / denominator;
    }
}
