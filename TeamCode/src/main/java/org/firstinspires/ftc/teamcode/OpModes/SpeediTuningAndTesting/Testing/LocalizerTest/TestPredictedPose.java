package org.firstinspires.ftc.teamcode.OpModes.SpeediTuningAndTesting.Testing.LocalizerTest;

import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.GoToPointConstants.velocityThreshold;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Follower.SpeediDrive;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;

/**
 * Empirical check for the physical braking predictor (used by go to point).
 * <p>
 * Drive around and let go of the sticks: the moment you release, the predicted stop pose is frozen,
 * and once the robot actually stops the error between prediction and reality is shown.
 * <p>
 * Open GoToPointConstants on the dashboard and adjust LIVE until the error stays near 0:
 * - robot stops BEFORE the predicted point (prediction too far) -> INCREASE the deceleration constant
 * - robot glides PAST the predicted point -> DECREASE the deceleration constant
 * Tune forwardDeceleration with forward releases and lateralDeceleration with sideways releases.
 */
@Config
@TeleOp(name = "Test Predicted Pose")
public class TestPredictedPose extends LinearOpMode {
    public static double minVelocityForTest = 30;//cm/s, releases below this speed are ignored

    SpeediDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //brake mode on, same conditions the predictor is used in
        drive = new SpeediDrive(hardwareMap, new Pose(), telemetry, true, true);
        drive.setRunMode(SpeediDrive.RunMode.MANUAL);

        boolean waitingForStop = false;
        Pose frozenPrediction = new Pose();
        double lastErrorX = 0, lastErrorY = 0, lastErrorTotal = 0;

        waitForStart();

        while (opModeIsActive()) {
            drive.drive(gamepad1);
            drive.update();

            boolean sticksReleased = Math.abs(gamepad1.left_stick_x) < 0.05 && Math.abs(gamepad1.left_stick_y) < 0.05 && Math.abs(gamepad1.right_stick_x) < 0.05;
            double speed = drive.localizer.getVelocity().getMagnitude();

            if (!waitingForStop && sticksReleased && speed >= minVelocityForTest) {
                frozenPrediction = drive.localizer.getPredictedPoseEstimate();
                waitingForStop = true;
            }
            if (waitingForStop && !sticksReleased) waitingForStop = false;//driver moved again, cancel
            if (waitingForStop && speed <= velocityThreshold) {
                Pose actual = drive.getCurrentPos();
                lastErrorX = actual.getX(DistanceUnit.CM) - frozenPrediction.getX(DistanceUnit.CM);
                lastErrorY = actual.getY(DistanceUnit.CM) - frozenPrediction.getY(DistanceUnit.CM);
                lastErrorTotal = Math.hypot(lastErrorX, lastErrorY);
                waitingForStop = false;
            }

            telemetry.addLine("Accelerate, then RELEASE the sticks and let the robot stop on its own");
            telemetry.addData("State", waitingForStop ? "measuring glide..." : "drive");
            telemetry.addData("Prediction error X (cm)", lastErrorX);
            telemetry.addData("Prediction error Y (cm)", lastErrorY);
            telemetry.addData("Prediction error total (cm)", lastErrorTotal);
            telemetry.addLine("Stopped before prediction -> INCREASE deceleration constant, glided past it -> DECREASE");
            telemetry.addData("Pose", drive.getCurrentPos());
            telemetry.addData("Predicted pose", drive.localizer.getPredictedPoseEstimate());
            telemetry.update();
        }
    }
}
