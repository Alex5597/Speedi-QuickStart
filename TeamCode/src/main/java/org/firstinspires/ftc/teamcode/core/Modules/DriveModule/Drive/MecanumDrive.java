package org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Drive;

import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.DriveCorrectionCoefficients.hPIDCoeff;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.DriveCorrectionCoefficients.hPIDCoeff_finalAdj;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.DriveCorrectionCoefficients.tPIDCoeff_GoToPoint;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.DriveCorrectionCoefficients.tPIDCoeff_Spline;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.DriveCorrectionCoefficients.tPIDCoeff_finalAdj;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.Forward;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.Heading;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.Lateral;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.WAIT_TIME_VARIABLE;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.forwardMultiplier;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.headingMultiplier;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.holdFinalPoint;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.lateralMultiplier;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.shouldUsePhysicalBraking;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.useDashboard;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.useFinalAdj;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.velocityThreshold;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Globals.isAuto;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Follower.SplineFollowing;
import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Localizer.Localizer;
import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Localizer.PinPointLocalizer;
import org.firstinspires.ftc.teamcode.core.Modules.Module;
import org.firstinspires.ftc.teamcode.core.Util.Algorithm.SplineGenerator.Spline;
import org.firstinspires.ftc.teamcode.core.Util.Algorithm.SquidController;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;
import org.firstinspires.ftc.teamcode.core.Util.Math.Vector;
import org.firstinspires.ftc.teamcode.core.Util.utils.DashboardPoseTracker;
import org.firstinspires.ftc.teamcode.core.Util.utils.DrawRobot;
import org.firstinspires.ftc.teamcode.core.Util.utils.Globals;

import java.util.LinkedList;
import java.util.Queue;

public class MecanumDrive implements Module {
    private static final SquidController tPid = new SquidController(tPIDCoeff_GoToPoint.p, tPIDCoeff_GoToPoint.i, tPIDCoeff_GoToPoint.d, 0);
    private static final PIDController hPid = new PIDController(hPIDCoeff.p, hPIDCoeff.i, hPIDCoeff.d);
    public PinPointLocalizer localizer;
    public Chassis motors;
    public boolean robotIsStuck = false;
    public SplineFollowing follower;
    public Spline curve = null;
    public DashboardPoseTracker poseTracker;
    Telemetry telemetry;
    HardwareMap hardwareMap;
    Vector powerVector;
    boolean timerResetedFailsafe = false, trajectoryDone = true;
    ElapsedTime timer = new ElapsedTime(), failsafeTimer = new ElapsedTime();
    Queue<Pose> targetPositions = new LinkedList<>();
    boolean isOnlyTarget = false;
    ElapsedTime timerSinceStart = new ElapsedTime();
    double startAngleTraj = 0;
    RunMode runMode = RunMode.MANUAL;
    boolean customTolerance = false;
    Pose tolerance = new Pose(), lastTolerance = new Pose(WAIT_TIME_VARIABLE, WAIT_TIME_VARIABLE, DistanceUnit.CM, WAIT_TIME_VARIABLE, AngleUnit.RADIANS);
    double noGoZoneTolerance = 0;
    boolean shouldWaitToStop = false, lastShouldWaitToStop = false;
    boolean noGoZone = false, willEnterNoGoZone = false, goingToNewTargetForAvoidingNoGoZone = false;
    Pose topLeftCorner = new Pose(), topRightCorner = new Pose(), bottomLeftCorner = new Pose(), bottomRightCorner = new Pose();
    private Pose lastTarget = new Pose();
    private Pose targetPose = new Pose();
    private Vector speedVector = new Vector(0, 0, 0);
    private int n;


    public MecanumDrive(HardwareMap hardwareMap, Pose startPose, Telemetry telemetry, boolean isAuto) {
        Globals.isAuto = isAuto;
        motors = new Chassis(hardwareMap, !shouldUsePhysicalBraking);
        localizer = new PinPointLocalizer(hardwareMap, startPose, telemetry);

        if (useDashboard) poseTracker = new DashboardPoseTracker(localizer);
        timer.reset();
        timerSinceStart.reset();
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        tPid.setPIDF(tPIDCoeff_GoToPoint.p, tPIDCoeff_GoToPoint.i, tPIDCoeff_GoToPoint.d, 0);
        hPid.setPIDF(hPIDCoeff.p, hPIDCoeff.i, hPIDCoeff.d, 0);

        tPid.reset();
        hPid.reset();

    }

    public MecanumDrive(HardwareMap hardwareMap, Telemetry telemetry, boolean isAuto) {
        Globals.isAuto = isAuto;
        motors = new Chassis(hardwareMap, !shouldUsePhysicalBraking);
        localizer = new PinPointLocalizer(hardwareMap, new Pose(), telemetry);

        if (useDashboard) poseTracker = new DashboardPoseTracker(localizer);
        timer.reset();
        timerSinceStart.reset();
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        tPid.setPIDF(tPIDCoeff_GoToPoint.p, tPIDCoeff_GoToPoint.i, tPIDCoeff_GoToPoint.d, 0);
        hPid.setPIDF(hPIDCoeff.p, hPIDCoeff.i, hPIDCoeff.d, 0);

        tPid.reset();
        hPid.reset();

    }

    public MecanumDrive(HardwareMap hardwareMap, Pose startPose, Telemetry telemetry, boolean brake, boolean isAuto) {
        Globals.isAuto = isAuto;
        motors = new Chassis(hardwareMap, brake);
        localizer = new PinPointLocalizer(hardwareMap, startPose, telemetry);
        if (useDashboard) poseTracker = new DashboardPoseTracker(localizer);
        timer.reset();
        timerSinceStart.reset();
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        tPid.setPIDF(tPIDCoeff_GoToPoint.p, tPIDCoeff_GoToPoint.i, tPIDCoeff_GoToPoint.d, 0);
        hPid.setPIDF(hPIDCoeff.p, hPIDCoeff.i, hPIDCoeff.d, 0);

        tPid.reset();
        hPid.reset();

    }
    public RunMode getRunMode() {
        return runMode;
    }
    public void setRunMode(RunMode runMode) {
        this.runMode = runMode;
    }

    public void setSpline_withInstantHeadingChange(Spline trajectory) {
        this.runMode = RunMode.Spline;
        this.curve = trajectory;
        localizer.update();
        follower = new SplineFollowing(localizer.getPoseEstimate(), trajectory, telemetry);
        targetPose = new Pose(trajectory.calculate(1), trajectory.heading(1));
        startAngleTraj = localizer.getPoseEstimate().getHeading(AngleUnit.RADIANS);
        trajectoryDone = false;
        robotIsStuck = false;
        timerSinceStart.reset();
        timerResetedFailsafe = false;
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
        localizer.update();
        follower = new SplineFollowing(localizer.getPoseEstimate(), trajectory, telemetry, Range.clip(rateOfChange, 0, 1));
        targetPose = new Pose(trajectory.calculate(1), trajectory.heading(1));
        robotIsStuck = false;
        startAngleTraj = localizer.getPoseEstimate().getHeading(AngleUnit.RADIANS);
        trajectoryDone = false;
        timerSinceStart.reset();
        timerResetedFailsafe = false;
    }

    public void setSpline_withTangentialHeadingChange(Spline trajectory) {
        this.runMode = RunMode.Spline;
        this.curve = trajectory;
        localizer.update();
        follower = new SplineFollowing(localizer.getPoseEstimate(), trajectory, telemetry);
        targetPose = new Pose(trajectory.calculate(1), trajectory.heading(1));
        robotIsStuck = false;
        startAngleTraj = localizer.getPoseEstimate().getHeading(AngleUnit.RADIANS);
        trajectoryDone = false;
        timerSinceStart.reset();
        timerResetedFailsafe = false;
    }

    public void setSpeedVector(Vector speedVector) {
        this.runMode = RunMode.MANUAL;
        this.speedVector = speedVector;
    }

    public void setTargetPose(Pose targetPose, boolean shouldWaitToStopCompletelyAtTheEndOfTrajectory) {
        this.shouldWaitToStop = shouldWaitToStopCompletelyAtTheEndOfTrajectory;
        this.runMode = RunMode.PID;
        this.targetPose = targetPose;
        targetPositions.clear();
        trajectoryDone = false;
        isOnlyTarget = true;
        robotIsStuck = false;
        startAngleTraj = localizer.getPoseEstimate().getHeading(AngleUnit.RADIANS);
        timerSinceStart.reset();
        timerResetedFailsafe = false;
        motors.setMinPowersToOvercomeFriction();

        tPid.setPIDF(tPIDCoeff_GoToPoint.p, tPIDCoeff_GoToPoint.i, tPIDCoeff_GoToPoint.d, 0);
        hPid.setPIDF(hPIDCoeff.p, hPIDCoeff.i, hPIDCoeff.d, 0);

        tPid.reset();
        hPid.reset();
    }

    public void setTargetPose(Pose targetPose, Pose tolerance, boolean shouldWaitToStopCompletelyAtTheEndOfTrajectory) {
        this.shouldWaitToStop = shouldWaitToStopCompletelyAtTheEndOfTrajectory;
        this.runMode = RunMode.PID;
        this.targetPose = targetPose;
        targetPositions.clear();
        trajectoryDone = false;
        isOnlyTarget = true;
        startAngleTraj = localizer.getPoseEstimate().getHeading(AngleUnit.RADIANS);
        timerSinceStart.reset();
        timerResetedFailsafe = false;
        customTolerance = true;
        robotIsStuck = false;
        this.tolerance = tolerance;

        motors.setMinPowersToOvercomeFriction();

        tPid.setPIDF(tPIDCoeff_GoToPoint.p, tPIDCoeff_GoToPoint.i, tPIDCoeff_GoToPoint.d, 0);
        hPid.setPIDF(hPIDCoeff.p, hPIDCoeff.i, hPIDCoeff.d, 0);

        tPid.reset();
        hPid.reset();
    }

    public void updateTargetPose(Pose targetPose, Pose tolerance, boolean shouldWaitToStopCompletelyAtTheEndOfTrajectory) {
        this.shouldWaitToStop = shouldWaitToStopCompletelyAtTheEndOfTrajectory;
        this.runMode = RunMode.PID;
        this.targetPose = targetPose;
        targetPositions.clear();
        trajectoryDone = false;
        isOnlyTarget = true;
        startAngleTraj = localizer.getPoseEstimate().getHeading(AngleUnit.RADIANS);
        customTolerance = true;
        robotIsStuck = false;
        timerSinceStart.reset();
        this.tolerance = tolerance;
        timerResetedFailsafe = false;
    }

    public void updateTargetPose(Pose targetPose, boolean shouldWaitToStopCompletelyAtTheEndOfTrajectory) {
        this.shouldWaitToStop = shouldWaitToStopCompletelyAtTheEndOfTrajectory;
        this.runMode = RunMode.PID;
        this.targetPose = targetPose;
        targetPositions.clear();
        trajectoryDone = false;
        isOnlyTarget = true;
        startAngleTraj = localizer.getPoseEstimate().getHeading(AngleUnit.RADIANS);
        customTolerance = false;
        robotIsStuck = false;
        timerSinceStart.reset();
        timerResetedFailsafe = false;
    }

    public void setTargetsList(Queue<Pose> targetPositions) {
        this.runMode = RunMode.PID;
        this.targetPositions = targetPositions;
        this.shouldWaitToStop = false;
        n = targetPositions.size();
        targetPose = targetPositions.poll();
        startAngleTraj = localizer.getPoseEstimate().getHeading(AngleUnit.RADIANS);
        trajectoryDone = false;
        isOnlyTarget = false;
        robotIsStuck = false;
        timerSinceStart.reset();
        timerResetedFailsafe = false;
    }

    /**
     * Strafes a distance on OX and OY relative to the robot
     *
     * @param distanceInCmLateral                            the distance to drive laterally (negative to drive left and positive to drive right)
     * @param distanceInCmForward                            the distance to drive forwardly (negative to drive back and positive to drive forward)
     * @param degreesToTurn                                  the number of degrees to turn
     * @param shouldWaitToStopCompletelyAtTheEndOfTrajectory either the robot should stop completely or not at the end of the strafe
     */
    public void driveRelativelyToRobotPos(double distanceInCmLateral, double distanceInCmForward, double degreesToTurn, boolean shouldWaitToStopCompletelyAtTheEndOfTrajectory) {
        setTargetPose(getCurrentPos().add(new Pose(distanceInCmLateral, distanceInCmForward, DistanceUnit.CM, degreesToTurn, AngleUnit.DEGREES).rotateFieldCoordinate(getCurrentPos().getHeading(AngleUnit.RADIANS))), shouldWaitToStopCompletelyAtTheEndOfTrajectory);
    }

    public boolean stopped() {
        if (!shouldWaitToStop) return true;
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
            if (runMode != RunMode.MANUAL) {
                if (!customTolerance) {
                    if (reachedTarget(3) && reachedHeading(3) && stopped()) {
                        if (isOnlyTarget)
                            trajectoryDone = true;
                        else {
                            targetPose = targetPositions.poll();
                            trajectoryDone = false;
                        }
                        robotIsStuck = false;
                    }
                } else if (getXError(DistanceUnit.CM) <= tolerance.getX(DistanceUnit.CM) && getYError(DistanceUnit.CM) <= tolerance.getY(DistanceUnit.CM) && reachedHeading(tolerance.getHeading(AngleUnit.RADIANS)) && stopped()) {
                    if (isOnlyTarget) {
                        trajectoryDone = true;
                        robotIsStuck = false;
                    } else {
                        targetPose = targetPositions.poll();
                        trajectoryDone = false;
                    }
                }

                if (goingToNewTargetForAvoidingNoGoZone && trajectoryDone) {
                    goingToNewTargetForAvoidingNoGoZone = false;
                    willEnterNoGoZone = false;
                    noGoZone = true;
                    if (!lastTolerance.equals(new Pose(WAIT_TIME_VARIABLE, WAIT_TIME_VARIABLE, DistanceUnit.CM, WAIT_TIME_VARIABLE, AngleUnit.RADIANS)))
                        updateTargetPose(lastTarget, lastTolerance, lastShouldWaitToStop);
                    else
                        updateTargetPose(lastTarget, lastShouldWaitToStop);
                    lastTolerance = new Pose(WAIT_TIME_VARIABLE, WAIT_TIME_VARIABLE, DistanceUnit.CM, WAIT_TIME_VARIABLE, AngleUnit.RADIANS);
                }

                if (!trajectoryDone) {
                    checkIfRobotIsStuck();
                    if (noGoZone) checkIfInsideNoGoZone(localizer.getPredictedPoseEstimate());
                }

                if (robotIsStuck) {
                    trajectoryDone = true;
                    motors.setMotorPower(0, 0, 0, 0);
                    powerVector = new Vector(0, 0, 0);
                } else {
                    if (!trajectoryDone || holdFinalPoint) updatePowerVector();
                    else motors.setMotorPower(0, 0, 0, 0);
                }
            }

            motors.update();
        } else {
            localizer.updateOnlyImu();
            motors.update();
        }
    }

    public void updatePowerVector() {
        Pose currentPose;
        if (!shouldUsePhysicalBraking) currentPose = localizer.getPoseEstimate();
        else currentPose = localizer.getPredictedPoseEstimate();

        switch (runMode) {
            case PID:
            case CalibrateSplinePID:
                if (willEnterNoGoZone) recalibrateTargetToAvoidNoGoZone();
                Vector err = targetPose.subtract(currentPose).toVec();
                err.setHeading(angleWrapper(err.getHeading()));

                if (reachedTarget(10) && reachedHeading(5) && runMode != RunMode.CalibrateSplinePID) {// && angleWrapper(err.getHeading()) <= Math.toRadians(5)) {
                    lateralMultiplier = 1;
                    headingMultiplier = 1;
                    forwardMultiplier = 1;
                    motors.resetMinPowersToOvercomeFriction();
                    if (useFinalAdj) {
                        err = targetPose.subtract(localizer.getPoseEstimate()).toVec();
                        tPid.setPIDF(tPIDCoeff_finalAdj.p, tPIDCoeff_finalAdj.i, tPIDCoeff_finalAdj.d, 0);
                        hPid.setPIDF(hPIDCoeff_finalAdj.p, hPIDCoeff_finalAdj.i, hPIDCoeff_finalAdj.d, 0);
                    }
                } else {
                    if (getXError(DistanceUnit.CM) <= 3) lateralMultiplier = Lateral;
                    if (getYError(DistanceUnit.CM) <= 3) forwardMultiplier = Forward;
                    if (angleWrapper(err.getHeading()) <= Math.toRadians(3))
                        headingMultiplier = Heading;
                    if (runMode != RunMode.CalibrateSplinePID) {
                        tPid.setPIDF(tPIDCoeff_GoToPoint.p, tPIDCoeff_GoToPoint.i, tPIDCoeff_GoToPoint.d, 0);
                        hPid.setPIDF(hPIDCoeff.p, hPIDCoeff.i, hPIDCoeff.d, 0);
                    } else {
                        motors.setMinPowersToOvercomeFriction();
                        tPid.setPIDF(tPIDCoeff_Spline.p, tPIDCoeff_Spline.i, tPIDCoeff_Spline.d, 0);
                        hPid.setPID(hPIDCoeff.p, hPIDCoeff.i, hPIDCoeff.d);
                    }
                }

                double distance = Math.hypot(err.getX(), err.getY());
                double calculatedCos = err.getX() / distance;
                double calculatedSin = err.getY() / distance;
                double translationalPower = tPid.calculate(-distance, 0);
                powerVector = new Vector(translationalPower * calculatedCos, translationalPower * calculatedSin);

                double headingDiff = angleWrapper(err.getHeading());
                double headingPower = -hPid.calculate(-headingDiff, 0);
                powerVector.setHeading(headingPower);
                powerVector = powerVector.rotate(currentPose.getHeading(AngleUnit.RADIANS)).scaleToMagnitude_AngularAsWell(1);
                if (runMode != RunMode.CalibrateSplinePID)
                    powerVector = new Vector(powerVector.getX() * lateralMultiplier, powerVector.getY() * forwardMultiplier, headingPower * headingMultiplier);

                motors.setMotorPower(powerVector);
                break;
            case Spline:
                Vector followerPower = follower.getMotorPower(currentPose, localizer.getVelocity(), localizer.getGlideVector());
                if (!followerPower.isNaN()) {
                    if (!followerPower.equals(new Vector(WAIT_TIME_VARIABLE, WAIT_TIME_VARIABLE))) {
                        powerVector = followerPower.rotate(currentPose.getHeading(AngleUnit.RADIANS));
                        motors.setMotorPower(powerVector);
                    } else updateTargetPose(targetPose, false);
                }
                break;
            case MANUAL:
                motors.setMotorPowerForced(speedVector);
                break;
        }
    }

    public void setMaxPower(double maxPower) {
        motors.setMaxPower(Range.clip(maxPower, 0, 1));
    }

    public boolean reachedTarget(double toleranceInCm) {
        if (runMode == RunMode.MANUAL || robotIsStuck) return true;
        Pose err;
        Pose robotPose = localizer.getPoseEstimate();
        if (targetPose.getX(DistanceUnit.CM) != WAIT_TIME_VARIABLE)
            err = targetPose.subtract(robotPose);
        else err = lastTarget.subtract(robotPose);
        return err.toVec().getMagnitude() <= toleranceInCm;
    }

    public boolean reachedHeading(double toleranceInDegrees) {
        if (runMode == RunMode.MANUAL) return true;
        Pose error;
        if (targetPose.getX(DistanceUnit.CM) != WAIT_TIME_VARIABLE)
            error = targetPose.subtract(localizer.getPoseEstimate());
        else error = lastTarget.subtract(localizer.getPoseEstimate());
        return Math.abs(Math.toDegrees(angleWrapper(error.getHeading(AngleUnit.RADIANS)))) <= toleranceInDegrees;
    }

    public double getXError(DistanceUnit distanceUnit) {
        Pose currPose = localizer.getPoseEstimate();
        return Math.abs(targetPose.getX(distanceUnit) - currPose.getX(distanceUnit));
    }

    public double getYError(DistanceUnit distanceUnit) {
        Pose currPose = localizer.getPoseEstimate();
        return Math.abs(targetPose.getY(distanceUnit) - currPose.getY(distanceUnit));
    }

    public double getHeadingError(AngleUnit angleUnit) {
        Pose currPose = localizer.getPoseEstimate();
        return Math.abs(angleWrapper(targetPose.getHeading(angleUnit) - currPose.getHeading(angleUnit)));
    }

    private void checkIfRobotIsStuck() {
        if (timerSinceStart.milliseconds() >= 1000 && localizer.getVelocity().getMagnitude() <= velocityThreshold && localizer.getVelocity().getHeading() <= 2 && !timerResetedFailsafe) {
            timerResetedFailsafe = true;
            failsafeTimer.reset();
        }
        if (timerResetedFailsafe && failsafeTimer.milliseconds() >= 300) {
            timerResetedFailsafe = false;
            localizer.update();
            if (localizer.getVelocity().getHeading() <= 2 && localizer.getVelocity().getMagnitude() <= velocityThreshold) {
                robotIsStuck = true;
            }
        }
    }

    public Pose getTarget() {
        return targetPose;
    }

    public boolean isDone() {
        if (runMode == RunMode.MANUAL || robotIsStuck) return true;
        return trajectoryDone;
    }

    public void errorTelemetry(boolean updated) {
        Pose error;
        if (targetPose.getX(DistanceUnit.CM) != WAIT_TIME_VARIABLE)
            error = targetPose.subtract(localizer.getPoseEstimate());
        else error = lastTarget.subtract(localizer.getPoseEstimate());
        targetTelemetry(true);
        telemetry.addData("error x", error.getX(DistanceUnit.CM));
        telemetry.addData("error y", error.getY(DistanceUnit.CM));
        telemetry.addData("error heading", error.getHeading(AngleUnit.DEGREES));

        telemetry.addData("Dist", Math.hypot(error.getX(DistanceUnit.CM), error.getY(DistanceUnit.CM)));
        telemetry.addData("is Done", (reachedTarget(3) && reachedHeading(2) && stopped()));
        telemetry.addData("Stopped", stopped());
        telemetry.addData("Target", (reachedTarget(3)));
        telemetry.addData("Target heading", (reachedHeading(2)));
        telemetry.addData("Condition", Math.abs(Math.toDegrees(angleWrapper(error.getHeading(AngleUnit.RADIANS)))));
        telemetry.addData("Robot is Stuck", robotIsStuck);
        if (!updated) telemetry.update();
    }

    public void targetTelemetry(boolean updated) {
        if (targetPose.getX(DistanceUnit.CM) != WAIT_TIME_VARIABLE) {
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
        return Math.abs(xPos - localizer.getPoseEstimate().getX(DistanceUnit.CM)) <= 0.2;///NU merge trebuie gasita formula
    }

    @Deprecated
    public boolean afterThisY(double yPos) {
        return (localizer.getPoseEstimate().getY(DistanceUnit.CM) - yPos) <= 2; ///NU merge trebuie gasita formula
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
        tPid.reset();
        hPid.reset();
    }

    public void setNoGoZone(Pose topLeftCorner, Pose bottomRightCorner, double noGoTolerance) {
        this.noGoZoneTolerance = noGoTolerance;
        this.topLeftCorner = topLeftCorner;
        this.topRightCorner = new Pose(bottomRightCorner.getX(DistanceUnit.CM), topLeftCorner.getY(DistanceUnit.CM), DistanceUnit.CM);
        this.bottomRightCorner = bottomRightCorner;
        this.bottomLeftCorner = new Pose(topLeftCorner.getX(DistanceUnit.CM), bottomRightCorner.getY(DistanceUnit.CM), DistanceUnit.CM);
        if (topLeftCorner.getX(DistanceUnit.CM) >= bottomRightCorner.getX(DistanceUnit.CM) || topLeftCorner.getY(DistanceUnit.CM) <= bottomRightCorner.getY(DistanceUnit.CM))
            throw new IllegalArgumentException("Zona no go definita gresit. Deschide desmos si verifica.");
        noGoZone = true;
        willEnterNoGoZone = false;
    }

    public void disableNoGoZone() {
        this.topLeftCorner = new Pose();
        this.topRightCorner = new Pose();
        this.bottomRightCorner = new Pose();
        this.bottomLeftCorner = new Pose();
        noGoZone = false;
        willEnterNoGoZone = false;
    }

    private boolean isInsideNoGoZone(Pose p) {
        double x = p.getX(DistanceUnit.CM);
        double y = p.getY(DistanceUnit.CM);

        double left = topLeftCorner.getX(DistanceUnit.CM);
        double right = bottomRightCorner.getX(DistanceUnit.CM);
        double top = topLeftCorner.getY(DistanceUnit.CM);
        double bottom = bottomRightCorner.getY(DistanceUnit.CM);

        return x >= left - noGoZoneTolerance && x <= right + noGoZoneTolerance &&
                y >= bottom - noGoZoneTolerance && y <= top + noGoZoneTolerance;
    }

    public void checkIfInsideNoGoZone(Pose positionToCheck) {
        if (!willEnterNoGoZone)
            if (isInsideNoGoZone(positionToCheck)) {
                willEnterNoGoZone = true;
            }
    }

    public void recalibrateTargetToAvoidNoGoZone() {
        willEnterNoGoZone = false;
        noGoZone = false;
        goingToNewTargetForAvoidingNoGoZone = true;
        double a = bottomRightCorner.distanceTo(localizer.getPoseEstimate(), DistanceUnit.CM);
        double b = bottomLeftCorner.distanceTo(localizer.getPoseEstimate(), DistanceUnit.CM);
        double c = topLeftCorner.distanceTo(localizer.getPoseEstimate(), DistanceUnit.CM);
        double d = topRightCorner.distanceTo(localizer.getPoseEstimate(), DistanceUnit.CM);

        double minDist = Math.min(Math.min(Math.min(a, b), c), d);
        Queue<Pose> targetPoses = new LinkedList<>();
        int numberOfTargets = countCornersBeforeDirect(localizer.getPoseEstimate(), targetPose, 10);

        if (minDist == a) {
            lastTarget = targetPose;
            if (customTolerance)
                lastTolerance = tolerance;
            lastShouldWaitToStop = shouldWaitToStop;
            bottomRightCorner.setHeading(targetPose.getHeading(AngleUnit.RADIANS), AngleUnit.RADIANS);
            targetPoses.add(bottomRightCorner);
            bottomRightCorner.setHeading(0, AngleUnit.RADIANS);
            if (numberOfTargets == 2) {
                if (targetPose.distanceTo(bottomLeftCorner, DistanceUnit.CM) < targetPose.distanceTo(topRightCorner, DistanceUnit.CM)) {
                    bottomLeftCorner.setHeading(targetPose.getHeading(AngleUnit.RADIANS), AngleUnit.RADIANS);
                    targetPoses.add(bottomLeftCorner);
                    bottomLeftCorner.setHeading(0, AngleUnit.RADIANS);
                } else {
                    topRightCorner.setHeading(targetPose.getHeading(AngleUnit.RADIANS), AngleUnit.RADIANS);
                    targetPoses.add(topRightCorner);
                    topRightCorner.setHeading(0, AngleUnit.RADIANS);
                }
            }

        } else if (minDist == b) {
            lastTarget = targetPose;
            if (customTolerance)
                lastTolerance = tolerance;
            lastShouldWaitToStop = shouldWaitToStop;
            bottomLeftCorner.setHeading(targetPose.getHeading(AngleUnit.RADIANS), AngleUnit.RADIANS);
            updateTargetPose(bottomLeftCorner, new Pose(5, 5, DistanceUnit.CM, 5, AngleUnit.DEGREES), false);
            bottomLeftCorner.setHeading(0, AngleUnit.RADIANS);
            if (numberOfTargets == 2) {
                if (targetPose.distanceTo(bottomRightCorner, DistanceUnit.CM) < targetPose.distanceTo(topRightCorner, DistanceUnit.CM)) {
                    bottomRightCorner.setHeading(targetPose.getHeading(AngleUnit.RADIANS), AngleUnit.RADIANS);
                    targetPoses.add(bottomRightCorner);
                    bottomRightCorner.setHeading(0, AngleUnit.RADIANS);
                } else {
                    topLeftCorner.setHeading(targetPose.getHeading(AngleUnit.RADIANS), AngleUnit.RADIANS);
                    targetPoses.add(topLeftCorner);
                    topLeftCorner.setHeading(0, AngleUnit.RADIANS);
                }
            }
        } else if (minDist == c) {
            lastTarget = targetPose;
            if (customTolerance)
                lastTolerance = tolerance;
            lastShouldWaitToStop = shouldWaitToStop;
            topLeftCorner.setHeading(targetPose.getHeading(AngleUnit.RADIANS), AngleUnit.RADIANS);
            updateTargetPose(topLeftCorner, new Pose(5, 5, DistanceUnit.CM, 5, AngleUnit.DEGREES), false);
            topLeftCorner.setHeading(0, AngleUnit.RADIANS);
            if (numberOfTargets == 2) {
                if (targetPose.distanceTo(bottomLeftCorner, DistanceUnit.CM) < targetPose.distanceTo(topRightCorner, DistanceUnit.CM)) {
                    bottomLeftCorner.setHeading(targetPose.getHeading(AngleUnit.RADIANS), AngleUnit.RADIANS);
                    targetPoses.add(bottomLeftCorner);
                    bottomLeftCorner.setHeading(0, AngleUnit.RADIANS);
                } else {
                    topRightCorner.setHeading(targetPose.getHeading(AngleUnit.RADIANS), AngleUnit.RADIANS);
                    targetPoses.add(topRightCorner);
                    topRightCorner.setHeading(0, AngleUnit.RADIANS);
                }
            }
        } else if (minDist == d) {
            lastTarget = targetPose;
            if (customTolerance)
                lastTolerance = tolerance;
            lastShouldWaitToStop = shouldWaitToStop;
            topRightCorner.setHeading(targetPose.getHeading(AngleUnit.RADIANS), AngleUnit.RADIANS);
            updateTargetPose(topRightCorner, new Pose(5, 5, DistanceUnit.CM, 5, AngleUnit.DEGREES), false);
            topRightCorner.setHeading(0, AngleUnit.RADIANS);
            if (numberOfTargets == 2) {
                if (targetPose.distanceTo(bottomRightCorner, DistanceUnit.CM) < targetPose.distanceTo(topLeftCorner, DistanceUnit.CM)) {
                    bottomRightCorner.setHeading(targetPose.getHeading(AngleUnit.RADIANS), AngleUnit.RADIANS);
                    targetPoses.add(bottomRightCorner);
                    bottomRightCorner.setHeading(0, AngleUnit.RADIANS);
                } else {
                    topLeftCorner.setHeading(targetPose.getHeading(AngleUnit.RADIANS), AngleUnit.RADIANS);
                    targetPoses.add(topLeftCorner);
                    topLeftCorner.setHeading(0, AngleUnit.RADIANS);
                }
            }
        }
        setTargetsList(targetPoses);
    }

    public int countCornersBeforeDirect(Pose start, Pose target, double clearanceCm) {
        if (!noGoZone) return 0;

        // Inflate no-go rectangle by clearance (robot radius + margin)
        double left = topLeftCorner.getX(DistanceUnit.CM) - clearanceCm;
        double right = bottomRightCorner.getX(DistanceUnit.CM) + clearanceCm;
        double top = topLeftCorner.getY(DistanceUnit.CM) + clearanceCm;
        double bottom = bottomRightCorner.getY(DistanceUnit.CM) - clearanceCm;

        // If start/target fall inside inflated rect, push them to nearest outside point (not a corner)
        Pose s = pointInRect(start, left, right, bottom, top) ? clampToOutside() : start;
        Pose t = pointInRect(target, left, right, bottom, top) ? clampToOutside() : target;

        // 1 corner needed? Try each expanded corner; if both legs are clear, 1 corner is enough
        final double EPS = 2; // small nudge to sit just outside the inflated rectangle
        Pose[] corners = new Pose[]{
                new Pose(left - EPS, top + EPS, DistanceUnit.CM), // TL
                new Pose(right + EPS, top + EPS, DistanceUnit.CM), // TR
                new Pose(right + EPS, bottom - EPS, DistanceUnit.CM), // BR
                new Pose(left - EPS, bottom - EPS, DistanceUnit.CM)  // BL
        };
        for (Pose c : corners) {
            if (!segmentIntersectsRect(s, c, left, right, bottom, top) &&
                    !segmentIntersectsRect(c, t, left, right, bottom, top)) {
                return 1;
            }
        }

        // Otherwise, 2 corners (wrapping around one side of the rectangle) will be sufficient
        return 2;
    }

    public double getError(DistanceUnit distanceUnit) {
        Pose err = targetPose.add(getCurrentPos());
        return Math.hypot(err.getX(distanceUnit), err.getY(distanceUnit));
    }

    public double getTrajectoryCount() {
        return n - targetPositions.size() + 1;
    }

    public void resetPosition(Pose startPose) {
        motors.setMotorPower(0, 0, 0, 0);
        localizer.resetPosition(startPose);

        targetPose = startPose;
        timer.reset();
        tPid.reset();
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
    private static boolean pointInRect(Pose p, double left, double right, double bottom, double top) {
        double x = p.getX(DistanceUnit.CM), y = p.getY(DistanceUnit.CM);
        return x > left && x < right && y > bottom && y < top; // strict interior
    }

    private static Pose clampToOutside() {
        throw new IllegalArgumentException("Target is inside no go zone");
    }

    private static boolean segmentIntersectsRect(Pose a, Pose b,
                                                 double left, double right, double bottom, double top) {
        // If either endpoint is strictly inside -> intersects
        if (pointInRect(a, left, right, bottom, top) || pointInRect(b, left, right, bottom, top))
            return true;

        Pose tl = new Pose(left, top, DistanceUnit.CM);
        Pose tr = new Pose(right, top, DistanceUnit.CM);
        Pose br = new Pose(right, bottom, DistanceUnit.CM);
        Pose bl = new Pose(left, bottom, DistanceUnit.CM);

        return segmentsIntersect(a, b, tl, tr) || // top
                segmentsIntersect(a, b, tr, br) || // right
                segmentsIntersect(a, b, br, bl) || // bottom
                segmentsIntersect(a, b, bl, tl);   // left
    }

    private static boolean segmentsIntersect(Pose p1, Pose q1, Pose p2, Pose q2) {
        int o1 = orientation(p1, q1, p2);
        int o2 = orientation(p1, q1, q2);
        int o3 = orientation(p2, q2, p1);
        int o4 = orientation(p2, q2, q1);
        if (o1 != o2 && o3 != o4) return true;
        if (o1 == 0 && onSegment(p1, p2, q1)) return true;
        if (o2 == 0 && onSegment(p1, q2, q1)) return true;
        if (o3 == 0 && onSegment(p2, p1, q2)) return true;
        return o4 == 0 && onSegment(p2, q1, q2);
    }

    private static int orientation(Pose a, Pose b, Pose c) {
        double val = (b.getY(DistanceUnit.CM) - a.getY(DistanceUnit.CM)) * (c.getX(DistanceUnit.CM) - b.getX(DistanceUnit.CM)) -
                (b.getX(DistanceUnit.CM) - a.getX(DistanceUnit.CM)) * (c.getY(DistanceUnit.CM) - b.getY(DistanceUnit.CM));
        if (Math.abs(val) < 1e-12) return 0;
        return (val > 0) ? 1 : 2;
    }

    private static boolean onSegment(Pose p, Pose q, Pose r) {
        double qx = q.getX(DistanceUnit.CM), qy = q.getY(DistanceUnit.CM);
        return qx >= Math.min(p.getX(DistanceUnit.CM), r.getX(DistanceUnit.CM)) - 1e-12 &&
                qx <= Math.max(p.getX(DistanceUnit.CM), r.getX(DistanceUnit.CM)) + 1e-12 &&
                qy >= Math.min(p.getY(DistanceUnit.CM), r.getY(DistanceUnit.CM)) - 1e-12 &&
                qy <= Math.max(p.getY(DistanceUnit.CM), r.getY(DistanceUnit.CM)) + 1e-12;
    }

    public enum RunMode {
        PID, MANUAL, Spline, CalibrateSplinePID
    }
}