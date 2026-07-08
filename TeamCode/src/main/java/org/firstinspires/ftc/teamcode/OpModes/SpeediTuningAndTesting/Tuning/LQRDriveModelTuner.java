package org.firstinspires.ftc.teamcode.OpModes.SpeediTuningAndTesting.Tuning;

import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.MecanumChassisConstants.minPowerToOvercomeKineticFriction;

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
 * Measures the feedforward drive model used by the LQR spline follower:
 * power = kS * sign(v) + kV * v + kA * a
 * <p>
 * Place the robot in the MIDDLE of a clear area: runs alternate direction and are capped at
 * maxRunDistance, so ~1.5m of free space is enough. The true max velocity does NOT need a long
 * runway: it is predicted from the model as (1 - kS) / kV and shown in telemetry.
 * Run once per axis (0 = forward, 1 = strafe, 2 = heading) and copy kS/kV/kA plus the suggested
 * acceleration/deceleration into Constants.LQRSplineConstants.
 */
@Config
@TeleOp(name = "LQR Drive Model Tuner")
public class LQRDriveModelTuner extends LinearOpMode {
    public static int axis = 0;
    public static double minPower = minPowerToOvercomeKineticFriction;
    public static double maxPower = 1;
    public static double powerStep = 0.15;
    public static double runSeconds = 0.75;
    public static double maxRunDistance = 100;//cm, a run ends early if the robot travels this far
    public static double settleSeconds = 0.55;
    public static double sampleDelaySeconds = 0.08;
    public static double velocityFilterAlpha = 0.65;
    public static double minVelocityForFit = 8.0;

    private SpeediDrive drive;
    private final ElapsedTime phaseTimer = new ElapsedTime();
    private final ElapsedTime totalTimer = new ElapsedTime();

    private int phase = 0;
    private int direction = 1;
    private double currentPower;
    private Pose runStartPose = new Pose();
    private double lastTime;
    private double lastVelocity;
    private double filteredVelocity;
    private double velocity;
    private double acceleration;
    private double maxVelocity;
    private double maxAcceleration;
    private double maxDeceleration;

    //normal equation sums for the least squares fit of power = kS*s + kV*v + kA*a, where s = sign of the command
    private int samples;
    private double sumSS, sumSV, sumSA;
    private double sumVV, sumVA, sumAA;
    private double sumPS, sumPV, sumPA;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SpeediDrive(hardwareMap, new Pose(), telemetry, true, true);
        drive.setRunMode(SpeediDrive.RunMode.MANUAL);

        telemetry.addLine("axis 0=forward, 1=strafe, 2=heading");
        telemetry.addLine("Place the robot in the middle of a clear area, runs alternate direction");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        currentPower = minPower;
        phaseTimer.reset();
        totalTimer.reset();
        lastTime = totalTimer.seconds();
        filteredVelocity = axisVelocity();
        lastVelocity = filteredVelocity;

        while (opModeIsActive()) {
            double command = commandForPhase();
            applyCommand(command);
            drive.update();
            updateModel(command);
            sendTelemetry(command);
        }
    }

    private double commandForPhase() {
        if (phase == 2) return 0;

        if (phase == 0) {
            if (phaseTimer.seconds() >= settleSeconds) {
                runStartPose = drive.getCurrentPos();
                phase = 1;
                phaseTimer.reset();
            }
            return 0;
        }

        boolean outOfRoom = axis != 2 && drive.getCurrentPos().distanceTo(runStartPose, DistanceUnit.CM) >= maxRunDistance;
        if (phaseTimer.seconds() >= runSeconds || outOfRoom) {
            direction = -direction;
            currentPower += powerStep;
            if (currentPower > maxPower + 1e-9) {
                phase = 2;
            } else {
                phase = 0;
            }
            phaseTimer.reset();
            return 0;
        }

        return currentPower * direction;
    }

    private void updateModel(double command) {
        double now = totalTimer.seconds();
        double dt = now - lastTime;
        if (dt <= 1e-4) return;

        double rawVelocity = axisVelocity();
        filteredVelocity = filteredVelocity + velocityFilterAlpha * (rawVelocity - filteredVelocity);
        velocity = filteredVelocity;
        acceleration = (filteredVelocity - lastVelocity) / dt;
        lastVelocity = filteredVelocity;
        lastTime = now;

        maxVelocity = Math.max(maxVelocity, Math.abs(velocity));
        if (command != 0) maxAcceleration = Math.max(maxAcceleration, Math.abs(acceleration));
        if (command == 0 && Math.signum(acceleration) != Math.signum(velocity)) {
            maxDeceleration = Math.max(maxDeceleration, Math.abs(acceleration));
        }

        //only fit while actually moving: near v=0 the friction sign is undefined and would bias kS
        if (phase == 1 && phaseTimer.seconds() >= sampleDelaySeconds && Math.abs(velocity) >= minVelocityForFit) {
            addSample(command, velocity, acceleration);
        }
    }

    private void addSample(double power, double velocity, double acceleration) {
        double sign = Math.signum(power);
        samples++;
        sumSS += sign * sign;
        sumSV += sign * velocity;
        sumSA += sign * acceleration;
        sumVV += velocity * velocity;
        sumVA += velocity * acceleration;
        sumAA += acceleration * acceleration;
        sumPS += power * sign;
        sumPV += power * velocity;
        sumPA += power * acceleration;
    }

    /**
     * @return {kS, kV, kA} from the 3x3 normal equations, or zeros while the system is still singular
     */
    private double[] solveModel() {
        double det = det3(sumSS, sumSV, sumSA, sumSV, sumVV, sumVA, sumSA, sumVA, sumAA);
        if (Math.abs(det) <= 1e-9) return new double[]{0, 0, 0};
        double kS = det3(sumPS, sumSV, sumSA, sumPV, sumVV, sumVA, sumPA, sumVA, sumAA) / det;
        double kV = det3(sumSS, sumPS, sumSA, sumSV, sumPV, sumVA, sumSA, sumPA, sumAA) / det;
        double kA = det3(sumSS, sumSV, sumPS, sumSV, sumVV, sumPV, sumSA, sumVA, sumPA) / det;
        return new double[]{kS, kV, kA};
    }

    private double det3(double a11, double a12, double a13,
                        double a21, double a22, double a23,
                        double a31, double a32, double a33) {
        return a11 * (a22 * a33 - a23 * a32) - a12 * (a21 * a33 - a23 * a31) + a13 * (a21 * a32 - a22 * a31);
    }

    private double axisVelocity() {
        Vector fieldVelocity = drive.localizer.getVelocity();
        if (axis == 2) return fieldVelocity.getHeading();

        Pose pose = drive.localizer.getPoseEstimate();
        Vector robotVelocity = fieldVelocity.rotate(pose.getHeading(AngleUnit.RADIANS));
        return axis == 1 ? robotVelocity.getX() : robotVelocity.getY();
    }

    private void applyCommand(double power) {
        if (axis == 2) {
            drive.motors.setMotorPowerForced(new Vector(0, 0, power));
        } else if (axis == 1) {
            drive.motors.setMotorPowerForced(new Vector(power, 0, 0));
        } else {
            drive.motors.setMotorPowerForced(new Vector(0, power, 0));
        }
    }

    private String axisName() {
        if (axis == 2) return "heading";
        if (axis == 1) return "strafe";
        return "forward";
    }

    private String phaseName() {
        if (phase == 2) return "done";
        if (phase == 1) return "running";
        return "settling";
    }

    private void sendTelemetry(double command) {
        double[] model = solveModel();
        telemetry.addData("Axis", axisName());
        telemetry.addData("Phase", phaseName());
        telemetry.addData("Command power", command);
        telemetry.addData("Current test power", currentPower);
        telemetry.addData("Velocity", velocity);
        telemetry.addData("Acceleration", acceleration);
        telemetry.addData("Samples", samples);
        telemetry.addLine("--- copy into LQRSplineConstants (" + axisName() + "KS/KV/KA) ---");
        telemetry.addData("kS", model[0]);
        telemetry.addData("kV", model[1]);
        telemetry.addData("kA", model[2]);
        telemetry.addLine("--- suggestions ---");
        telemetry.addData("Predicted TRUE max velocity (1-kS)/kV", model[1] > 1e-9 ? (1.0 - model[0]) / model[1] : 0);
        telemetry.addData("Observed max velocity (short runs)", maxVelocity);
        telemetry.addData("maxAcceleration suggestion (~80% of)", maxAcceleration);
        telemetry.addData("maxDeceleration suggestion (~80% of)", maxDeceleration);
        telemetry.addData("Use units", axis == 2 ? "rad/s and rad/s^2" : "cm/s and cm/s^2");
        telemetry.update();
    }
}
