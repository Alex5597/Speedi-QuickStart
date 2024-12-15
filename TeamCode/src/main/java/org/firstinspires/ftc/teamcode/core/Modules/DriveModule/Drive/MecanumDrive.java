package org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Drive;

import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.DriveCorrectionCoefficients.hPIDCoeff;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.DriveCorrectionCoefficients.hPIDCoeff_finalAdj;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.DriveCorrectionCoefficients.xPIDCoeff;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.DriveCorrectionCoefficients.xPIDCoeff_finalAdj;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.DriveCorrectionCoefficients.yPIDCoeff;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.DriveCorrectionCoefficients.yPIDCoeff_finalAdj;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.Forward;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.Heading;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.Lateral;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.WAIT_TIME_VARIABLE;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.forwardMultiplier;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.headignMultiplier;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.lateralMultiplier;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.resetMultipliers;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.useDashboard;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.velocityThreshold;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Globals.isAuto;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Follower.SplineFollowing;
import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Localizer.Localizer;
import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Localizer.PinPointLocalizer;
import org.firstinspires.ftc.teamcode.core.Modules.Module;
import org.firstinspires.ftc.teamcode.core.Modules.OutTake_Intake.LinearSlides;
import org.firstinspires.ftc.teamcode.core.Util.Algorithm.SplineGenerator.Spline;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;
import org.firstinspires.ftc.teamcode.core.Util.Math.Vector;
import org.firstinspires.ftc.teamcode.core.Util.utils.DashboardPoseTracker;
import org.firstinspires.ftc.teamcode.core.Util.utils.DrawRobot;
import org.firstinspires.ftc.teamcode.core.Util.utils.Globals;

import java.util.LinkedList;
import java.util.Queue;

public class MecanumDrive implements Module {
    Telemetry telemetry;
    HardwareMap hardwareMap;
    public PinPointLocalizer localizer;
    public Chassis motors;
    private Pose targetPose = new Pose(), lastTarget = new Pose();
    private Vector speedVector = new Vector(0, 0, 0);
    Vector powerVector;
    boolean timerResetedFailsafe = false, trajectoryDone = true, waitingTimer = false;
    ElapsedTime timer = new ElapsedTime(), failsafeTimer = new ElapsedTime();
    Queue<Pose> targetPositions = new LinkedList<>();
    boolean isOnlyTarget = false;
    private static PIDController xPid = new PIDController(xPIDCoeff.p, xPIDCoeff.i, xPIDCoeff.d);
    private static PIDController yPid = new PIDController(yPIDCoeff.p, yPIDCoeff.i, yPIDCoeff.d);
    private static PIDController hPid = new PIDController(hPIDCoeff.p, hPIDCoeff.i, hPIDCoeff.d);
    ElapsedTime timerSinceStart = new ElapsedTime();
    public boolean robotIsStuck = false;
    private int n;
    double startAngleTraj = 0;

    public enum RunMode {
        PID, MANUAL, Spline
    }

    public SplineFollowing follower;
    RunMode runMode = RunMode.MANUAL;

    public RunMode getRunMode() {
        return runMode;
    }

    public Spline curve = null;
    public DashboardPoseTracker poseTracker;
    boolean customTolerance = false;
    Pose tolerance = new Pose();

    public MecanumDrive(HardwareMap hardwareMap, Pose startPose, Telemetry telemetry, boolean isAuto) {
        Globals.isAuto = isAuto;
        motors = new Chassis(hardwareMap, true);
        localizer = new PinPointLocalizer(hardwareMap, startPose, telemetry);

        if (useDashboard)
            poseTracker = new DashboardPoseTracker(localizer);
        timer.reset();
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        xPid.setPID(xPIDCoeff.p, xPIDCoeff.i, xPIDCoeff.d);
        yPid.setPID(yPIDCoeff.p, yPIDCoeff.i, yPIDCoeff.d);
        hPid.setPID(hPIDCoeff.p, hPIDCoeff.i, hPIDCoeff.d);

        xPid.reset();
        yPid.reset();
        hPid.reset();

    }

    public MecanumDrive(HardwareMap hardwareMap, Telemetry telemetry, boolean isAuto) {
        Globals.isAuto = isAuto;

        motors = new Chassis(hardwareMap, true);
        localizer = new PinPointLocalizer(hardwareMap, new Pose(), telemetry);

        if (useDashboard)
            poseTracker = new DashboardPoseTracker(localizer);
        timer.reset();
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        xPid.setPID(xPIDCoeff.p, xPIDCoeff.i, xPIDCoeff.d);
        yPid.setPID(yPIDCoeff.p, yPIDCoeff.i, yPIDCoeff.d);
        hPid.setPID(hPIDCoeff.p, hPIDCoeff.i, hPIDCoeff.d);

        xPid.reset();
        yPid.reset();
        hPid.reset();

    }

    public MecanumDrive(HardwareMap hardwareMap, Pose startPose, Telemetry telemetry, boolean brake, boolean isAuto) {
        Globals.isAuto = isAuto;
        motors = new Chassis(hardwareMap, brake);
        localizer = new PinPointLocalizer(hardwareMap, startPose, telemetry);

        if (useDashboard)
            poseTracker = new DashboardPoseTracker(localizer);
        timer.reset();
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        xPid.setPID(xPIDCoeff.p, xPIDCoeff.i, xPIDCoeff.d);
        yPid.setPID(yPIDCoeff.p, yPIDCoeff.i, yPIDCoeff.d);
        hPid.setPID(hPIDCoeff.p, hPIDCoeff.i, hPIDCoeff.d);

        xPid.reset();
        yPid.reset();
        hPid.reset();

    }

    public void setSpline_withInstantHeadingChange(Spline trajectory) {
        this.runMode = RunMode.Spline;
        this.curve = trajectory;
        follower = new SplineFollowing(localizer, motors, trajectory, telemetry);
        targetPose = new Pose(trajectory.calculate(1), trajectory.heading(1));
        startAngleTraj = localizer.getPoseEstimate().getHeading();
        trajectoryDone = false;
    }

    /**
     * Sets a spline that should be followed with a slower heading change
     *
     * @param trajectory   the trajectory to follow
     * @param rateOfChange variable between 0 and 1 that determines the rate of change of the heading(look into linear interpolation for further details)
     */
    public void setSpline_withSlowerHeadingChange(Spline trajectory, double rateOfChange) {
        this.runMode = RunMode.Spline;
        this.curve = trajectory;
        follower = new SplineFollowing(localizer, motors, trajectory, telemetry, Range.clip(rateOfChange, 0, 1));
        targetPose = new Pose(trajectory.calculate(1), trajectory.heading(1));
        startAngleTraj = localizer.getPoseEstimate().getHeading();
        trajectoryDone = false;
    }

    public void setRunMode(RunMode runMode) {
        this.runMode = runMode;
    }

    public void setSpeedVector(Vector speedVector) {
        this.runMode = RunMode.MANUAL;
        this.speedVector = speedVector;
    }

    public void setTargetPose(Pose targetPose) {
        this.runMode = RunMode.PID;
        this.targetPose = targetPose;
        targetPositions.clear();
        trajectoryDone = false;
        isOnlyTarget = true;
        startAngleTraj = localizer.getPoseEstimate().getHeading();
        timerSinceStart.reset();
    }

    public void setTargetPose(Pose targetPose, Pose tolerance) {
        this.runMode = RunMode.PID;
        this.targetPose = targetPose;
        targetPositions.clear();
        trajectoryDone = false;
        isOnlyTarget = true;
        startAngleTraj = localizer.getPoseEstimate().getHeading();
        timerSinceStart.reset();
        customTolerance = true;
        this.tolerance = tolerance;
    }

    public void setTargetsList(Queue<Pose> targetPositions) {
        this.runMode = RunMode.PID;
        this.targetPositions = targetPositions;
        n = targetPositions.size();
        targetPose = targetPositions.poll();
        startAngleTraj = localizer.getPoseEstimate().getHeading();
        trajectoryDone = false;
        isOnlyTarget = false;
        timerSinceStart.reset();
    }

    public boolean stopped() {
        return localizer.getVelocity().getMagnitude() <= velocityThreshold;
    }

    @Override
    public void update() {
        if (isAuto) {
            localizer.update();

            if (useDashboard) {
                poseTracker.update();
                DrawRobot.drawDebug(this);
            }
            if (!trajectoryDone) {
                if (!customTolerance) {
                    if (runMode != RunMode.MANUAL && (
                            (reachedTarget(4) && reachedHeading(4) && !waitingTimer //&& stopped()
                                    || isStuck()))) {
                        motors.setMotorPower(new Vector(0, 0, 0));
                        customTolerance = false;
                        tolerance = new Pose();
                        resetMultipliers();
                        motors.setMinPowersToOvercomeFriction();

                        if (!targetPositions.isEmpty()) {
                            timerSinceStart.reset();
                            lastTarget = targetPose;
                            startAngleTraj = localizer.getPoseEstimate().getHeading();
                            targetPose = targetPositions.poll();
                        } else {
                            xPid.reset();
                            yPid.reset();
                            hPid.reset();
                            motors.setMaxPower(1);

                            trajectoryDone = true;
                        }
                    } else
                        updatePowerVector();
                } else if ((getXError() <= tolerance.getX() && getYError() <= tolerance.getY() && reachedHeading(tolerance.getHeading()))) {
                    motors.setMotorPower(new Vector(0, 0, 0));
                    customTolerance = false;
                    tolerance = new Pose();
                    resetMultipliers();
                    motors.setMinPowersToOvercomeFriction();

                    if (!targetPositions.isEmpty()) {
                        timerSinceStart.reset();
                        lastTarget = targetPose;
                        startAngleTraj = localizer.getPoseEstimate().getHeading();
                        targetPose = targetPositions.poll();
                    } else {
//                        xPid.reset();
//                        yPid.reset();
//                        hPid.reset();
                        motors.setMaxPower(1);

                        trajectoryDone = true;
                        telemetry.addLine("A intrat pe aici");
                        telemetry.update();
                    }
                } else
                    updatePowerVector();
                if (robotIsStuck) {
                    robotIsStuck = false;
                    trajectoryDone = true;
                }
                motors.update();
            }
        } else {
            localizer.updateOnlyImu();
            motors.update();
        }
    }

    public void updatePowerVector() {
        switch (runMode) {
            case PID:
                if (targetPose.getX() == WAIT_TIME_VARIABLE) {
                    motors.setMotorPower(new Vector(0, 0, 0));
                    resetMultipliers();
                    if (!waitingTimer) {
                        waitingTimer = true;
                        timer.reset();
                    }
                    if (waitingTimer && timer.milliseconds() >= targetPose.getY())
                        waitingTimer = false;
                } else {
                    //Pose currentPose = localizer.getPredictedPoseEstimate();
                    Pose currentPose = localizer.getPoseEstimate();
                    Vector err = targetPose.subtract(currentPose).toVec().rotate(currentPose.getHeading());


                    if (getXError() <= 5) {
                        lateralMultiplier = Lateral;
                        xPid.setPID(xPIDCoeff_finalAdj.p, xPIDCoeff_finalAdj.i, xPIDCoeff_finalAdj.d);
                    } else
                        xPid.setPID(xPIDCoeff.p, xPIDCoeff.i, xPIDCoeff.d);
                    if (getYError() <= 5) {
                        forwardMultiplier = Forward;
                        yPid.setPID(yPIDCoeff_finalAdj.p, yPIDCoeff_finalAdj.i, yPIDCoeff_finalAdj.d);
                    } else
                        yPid.setPID(yPIDCoeff.p, yPIDCoeff.i, yPIDCoeff.d);
//                    if (Math.abs(angleWrapper(err.getHeading())) <= Math.toRadians(5)) {
//                        headignMultiplier = Heading;
//                        hPid.setPID(hPIDCoeff_finalAdj.p, hPIDCoeff_finalAdj.i, hPIDCoeff_finalAdj.d);
//                    } else
                    hPid.setPID(hPIDCoeff.p, hPIDCoeff.i, hPIDCoeff.d);

                    double xPower = xPid.calculate(-err.getX(), 0);
                    double yPower = yPid.calculate(-err.getY(), 0);
                    powerVector = new Vector(xPower, yPower);
                    double headingDiff = angleWrapper(err.getHeading());
                    double headingPower = -hPid.calculate(-headingDiff, 0);
                    powerVector = new Vector(powerVector.getX(), powerVector.getY(), headingPower);
                    if (Math.abs(powerVector.getX()) + Math.abs(powerVector.getY()) + Math.abs(powerVector.getHeading()) > 1.2)
                        powerVector = powerVector.scaleToMagnitude_AngularAsWell(1.2);

                    powerVector = new Vector(powerVector.getX() * lateralMultiplier, powerVector.getY() * forwardMultiplier, headingPower * headignMultiplier);
                    //motors.setMotorPower(new Vector[]{new Vector(powerVector.getX(), powerVector.getY() + powerVector.getHeading()), new Vector(powerVector.getX(), powerVector.getY() - powerVector.getHeading())});
                    motors.setMotorPower(powerVector);
                }
                break;
            case Spline:
                Vector power = follower.getMotorPower();
                if (!power.equals(new Vector(WAIT_TIME_VARIABLE, WAIT_TIME_VARIABLE)))
                    //motors.setMotorPower(new Vector[]{new Vector(powerVector.getX(), powerVector.getY() + powerVector.getHeading()), new Vector(powerVector.getX(), powerVector.getY() - powerVector.getHeading())});
                    motors.setMotorPower(power);
                else
                    setTargetPose(targetPose);
                break;
            case MANUAL:
                motors.setMotorPower(speedVector);
                break;
        }
    }

    public void setMaxPower(double maxPower) {
        motors.setMaxPower(Range.clip(maxPower, 0, 1));
    }

    public boolean reachedTarget(double toleranceInCm) {
        if (runMode == RunMode.MANUAL) return true;
        Pose err;
        Pose robotPose = localizer.getPoseEstimate();
        if (targetPose.getX() != WAIT_TIME_VARIABLE)
            err = targetPose.subtract(robotPose);
        else err = lastTarget.subtract(robotPose);
        return err.toVec().getMagnitude() <= toleranceInCm;
    }


    public boolean reachedHeading(double toleranceInDegrees) {
        if (runMode == RunMode.MANUAL) return true;
        Pose error;
        if (targetPose.getX() != WAIT_TIME_VARIABLE)
            error = targetPose.subtract(localizer.getPoseEstimate());
        else error = lastTarget.subtract(localizer.getPoseEstimate());
        return Math.abs(Math.toDegrees(angleWrapper(error.getHeading()))) <= toleranceInDegrees;
    }

    public double getXError() {
        Pose currPose = localizer.getPoseEstimate();
        return Math.abs(targetPose.getX() - currPose.getX());
    }

    public double getYError() {
        Pose currPose = localizer.getPoseEstimate();
        return Math.abs(targetPose.getY() - currPose.getY());
    }

    public double getHeadingError() {
        Pose currPose = localizer.getPoseEstimate();
        return Math.abs(angleWrapper(targetPose.getHeading() - currPose.getHeading()));
    }

    public boolean isStuck() {
        if (timerSinceStart.seconds() >= 1 && localizer.getVelocity().getMagnitude() <= velocityThreshold && !timerResetedFailsafe) {
            timerResetedFailsafe = true;
            failsafeTimer.reset();
        }
        if (timerResetedFailsafe && failsafeTimer.milliseconds() >= 800) {
            timerResetedFailsafe = false;
            if (stopped()) {
                robotIsStuck = true;
                return true;
            }
        }
        return false;
    }

    public Pose getTarget() {
        return targetPose;
    }

    public boolean isDone() {
        if (runMode == RunMode.MANUAL) return true;
        return trajectoryDone;
    }

    public void errorTelemetry(boolean updated) {
        Pose error;
        if (targetPose.getX() != WAIT_TIME_VARIABLE)
            error = targetPose.subtract(localizer.getPoseEstimate());
        else
            error = lastTarget.subtract(localizer.getPoseEstimate());
        targetTelemetry(true);
        telemetry.addData("error x", error.getX());
        telemetry.addData("error y", error.getY());
        telemetry.addData("error heading", error.getHeading());

        telemetry.addData("Dist", Math.hypot(error.getX(), error.getY()));
        telemetry.addData("is Done", (reachedTarget(3) && reachedHeading(2) && stopped()));
        telemetry.addData("Stopped", stopped());
        telemetry.addData("Target", (reachedTarget(3)));
        telemetry.addData("Target heading", (reachedHeading(2)));
        telemetry.addData("Condition", Math.abs(Math.toDegrees(angleWrapper(error.getHeading()))));
        if (!updated) telemetry.update();
    }

    public void targetTelemetry(boolean updated) {
        if (targetPose.getX() != WAIT_TIME_VARIABLE) {
            telemetry.addData("Target", targetPose.toString());
        } else {
            telemetry.addData("Target", lastTarget.toString());
        }
        if (!updated) telemetry.update();
    }

    public void currentPosTelemetry(boolean updated) {
        Pose currPos = getCurrentPos();
        telemetry.addData("POS", currPos.toString());
        telemetry.addData("Predicted pos", localizer.getPredictedPoseEstimate().toString());
        if (!updated) telemetry.update();
    }

    public void PinPointErrorTelemetry(boolean updated) {
        telemetry.addData("Frequency", localizer.odo.getFrequency());
        telemetry.addData("Loop Time", localizer.odo.getLoopTime());
        telemetry.addData("Status", localizer.odo.getDeviceStatus());
        telemetry.addData("EncX value", localizer.odo.getEncoderX());
        telemetry.addData("EncY value", localizer.odo.getEncoderY());
        telemetry.addData("Pos X value", localizer.odo.getPosX());
        telemetry.addData("Pos Y value", localizer.odo.getPosY());
        telemetry.addData("Vel X value", localizer.odo.getVelX() / 10.00);
        telemetry.addData("Vel Y value", localizer.odo.getVelY() / 10.00);

        if (!updated) telemetry.update();
    }

    @Deprecated
    public boolean afterThisX(double xPos) {
        return Math.abs(xPos - localizer.getPoseEstimate().getX()) <= 0.2;///NU merge trebuie gasita formula
    }

    @Deprecated
    public boolean afterThisY(double yPos) {
        return (localizer.getPoseEstimate().getY() - yPos) <= 2; ///NU merge trebuie gasita formula
    }

    @Deprecated
    public boolean afterThisTime(double timeInMilis) {
        return timerSinceStart.milliseconds() >= timeInMilis;
    }

    public void stop() {
        motors.setMotorPower(0, 0, 0, 0);
        n = 0;
        targetPositions.clear();
        trajectoryDone = true;
    }

    public Pose getCurrentPos() {
        return localizer.getPoseEstimate();
    }

    public void hardResetPos(Pose newPose) {
        targetPose = newPose;
        motors.setMotorPower(0, 0, 0, 0);
        localizer.resetPosition(newPose);
        timer.reset();
        xPid.reset();
        yPid.reset();
        hPid.reset();
    }

    public double getError() {
        Pose err = targetPose.add(getCurrentPos());
        return Math.hypot(err.getX(), err.getY());
    }

    public double getTrajectoryCount() {
        return n - targetPositions.size() + 1;
    }

    public void resetPosition(Pose startPose) {
        motors.setMotorPower(0, 0, 0, 0);
        localizer.resetPosition(startPose);

        targetPose = startPose;
        timer.reset();
        xPid.reset();
        yPid.reset();
        hPid.reset();
    }

    private double angleWrapper(double angle) {
        angle %= (2.0 * Math.PI);
        if (angle > Math.PI) angle -= 2.0 * Math.PI;
        if (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    public Localizer getLocalizerInstance() {
        return localizer;
    }
}
/*
                    Vector[] finalPower = new Vector[]{new Vector(0, 0), new Vector(0, 0)};

                    Pose currentPose = localizer.getPredictedPoseEstimate();
                    Pose err = targetPose.subtract(currentPose);

                    xPid.setPID(xPIDCoeff.p, xPIDCoeff.i, xPIDCoeff.d);
                    yPid.setPID(yPIDCoeff.p, yPIDCoeff.i, yPIDCoeff.d);
                    double xPower = xPid.calculate(-err.getX(), 0);
                    double yPower = yPid.calculate(-err.getY(), 0);
                    Vector correctionVector = new Vector(xPower, yPower);

                    if (correctionVector.getMagnitude() >= 1) {
                        if (Math.abs(angleWrapper(err.getHeading())) >= Math.toRadians(25))
                            correctionVector = correctionVector.scaleToMagnitude(0.85);
                        else
                            correctionVector = correctionVector.scaleToMagnitude(1);
                    }
                    finalPower[0] = correctionVector;
                    finalPower[1] = correctionVector;

                    double headingDiff = angleWrapper(err.getHeading());
                    hPid.setPID(hPIDCoeff.p, hPIDCoeff.i, hPIDCoeff.d);
                    double headingPower = -hPid.calculate(-headingDiff, 0);//* (Math.abs(headingDiff) >= Math.toRadians(5) ? 2 : 1);
                    Vector headingVector = new Vector(1, 1).rotate(Math.toRadians(-90)).scalarMultiply(Range.clip(headingPower, -1, 1)).rotate(-currentPose.getHeading());

                    Vector leftSideVector = finalPower[0].add(headingVector);
                    Vector rightSideVector = finalPower[1].subtract(headingVector);
                    if (leftSideVector.getMagnitude() > 1 || rightSideVector.getMagnitude() > 1) {
                        double scalar = Math.min(Vector.findScaleFactor(leftSideVector, headingVector), Vector.findScaleFactor(rightSideVector, headingVector)); //TODO POSIBIL max IN LOC DE min
                        headingVector = headingVector.scalarMultiply(scalar);
                        motors.setMotorPower(new Vector[]{finalPower[0].add(headingVector), finalPower[1].subtract(headingVector)}, currentPose.getHeading());
                        return;
                    } else {
                        finalPower[0] = finalPower[0].add(headingVector);
                        finalPower[1] = finalPower[1].subtract(headingVector);
                    }
                    motors.setMotorPower(finalPower, currentPose.getHeading());
                    */