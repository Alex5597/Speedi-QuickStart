package org.firstinspires.ftc.teamcode.OpModes.SpeediTuningAndTesting.Tuning;

import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.GoToPointConstants.velocityThreshold;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Follower.SpeediDrive;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;
import org.firstinspires.ftc.teamcode.core.Util.Math.Vector;

/**
 * Measures the braking deceleration used by the physical braking glide predictor
 * (GoToPointConstants.forwardDeceleration / lateralDeceleration).
 * <p>
 * It accelerates, cuts power and measures the glide with the SAME formula the predictor uses
 * (deceleration = v^2 / (2 * glide distance)), so the result is directly consistent with it.
 * Runs alternate direction so only ~1.5m of clear space is needed. Repeats are averaged.
 * <p>
 * axis 0 = forward, 1 = strafe. After copying the value, verify it live with "Test Predicted Pose".
 */
@Config
@TeleOp(name = "Braking Deceleration Tuner")
public class BrakingDecelerationTuner extends LinearOpMode {
    public static int axis = 0;
    public static double power = 0.7;
    public static double runSeconds = 1.0;
    public static double maxRunDistance = 100;//cm, the run ends early if the robot travels this far
    public static double settleSeconds = 0.5;
    public static int repeats = 4;
    public static double minVelocityForMeasurement = 40;//cm/s, a braking sample only counts above this speed

    private SpeediDrive drive;
    private final ElapsedTime phaseTimer = new ElapsedTime();

    private int phase = 0;//0 settle, 1 run, 2 brake, 3 done
    private int direction = 1;
    private int runsDone = 0;
    private int samplesUsed = 0;
    private double decelerationSum = 0;
    private double lastMeasuredDeceleration = 0;
    private double brakeStartVelocity = 0;
    private Pose runStartPose = new Pose();
    private Pose brakeStartPose = new Pose();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //brake mode on: the glide predictor is meant to work with physical braking
        drive = new SpeediDrive(hardwareMap, new Pose(), telemetry, true, true);
        drive.setRunMode(SpeediDrive.RunMode.MANUAL);

        telemetry.addLine("axis 0 = forward, 1 = strafe");
        telemetry.addLine("Place the robot in the middle of a clear lane (~1.5m each way)");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        phaseTimer.reset();

        while (opModeIsActive()) {
            drive.update();
            switch (phase) {
                case 0://settle
                    applyPower(0);
                    if (phaseTimer.seconds() >= settleSeconds) {
                        runStartPose = drive.getCurrentPos();
                        phase = 1;
                        phaseTimer.reset();
                    }
                    break;
                case 1://accelerate
                    applyPower(power * direction);
                    if (phaseTimer.seconds() >= runSeconds || distanceFrom(runStartPose) >= maxRunDistance) {
                        brakeStartVelocity = Math.abs(axisVelocity());
                        brakeStartPose = drive.getCurrentPos();
                        applyPower(0);
                        phase = 2;
                        phaseTimer.reset();
                    }
                    break;
                case 2://braking, wait for full stop and measure the glide
                    applyPower(0);
                    if (drive.localizer.getVelocity().getMagnitude() <= velocityThreshold) {
                        double glide = distanceFrom(brakeStartPose);
                        if (glide >= 1 && brakeStartVelocity >= minVelocityForMeasurement) {
                            lastMeasuredDeceleration = brakeStartVelocity * brakeStartVelocity / (2.0 * glide);
                            decelerationSum += lastMeasuredDeceleration;
                            samplesUsed++;
                        }
                        runsDone++;
                        direction = -direction;
                        phase = runsDone >= repeats ? 3 : 0;
                        phaseTimer.reset();
                    }
                    break;
                case 3://done
                    applyPower(0);
                    break;
            }
            sendTelemetry();
        }
    }

    private void applyPower(double value) {
        if (axis == 1) drive.motors.setMotorPowerForced(new Vector(value, 0, 0));
        else drive.motors.setMotorPowerForced(new Vector(0, value, 0));
    }

    private double axisVelocity() {
        Vector robotVelocity = drive.localizer.getVelocity().rotate(drive.getCurrentPos().getHeading(AngleUnit.RADIANS));
        return axis == 1 ? robotVelocity.getX() : robotVelocity.getY();
    }

    private double distanceFrom(Pose reference) {
        return drive.getCurrentPos().distanceTo(reference, DistanceUnit.CM);
    }

    private void sendTelemetry() {
        telemetry.addData("Axis", axis == 1 ? "strafe (lateralDeceleration)" : "forward (forwardDeceleration)");
        telemetry.addData("Phase", phase == 3 ? "DONE" : (phase == 2 ? "braking" : (phase == 1 ? "running" : "settling")));
        telemetry.addData("Runs done", runsDone + "/" + repeats);
        telemetry.addData("Velocity at brake", brakeStartVelocity);
        telemetry.addData("Last measured deceleration", lastMeasuredDeceleration);
        telemetry.addData("AVERAGE deceleration (copy this)", samplesUsed > 0 ? decelerationSum / samplesUsed : 0);
        telemetry.addLine(phase == 3 ? "Copy the average into GoToPointConstants, then verify with Test Predicted Pose" : "");
        telemetry.update();
    }
}
