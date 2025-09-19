package org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Follower;

import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.FollowerConstants.tPIDCoeff_SplineFollower;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.GoToPointConstants.hPIDCoeff_GoToPoint;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.GoToPointConstants.hPIDCoeff_finalAdj;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.GoToPointConstants.holdFinalPoint;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.GoToPointConstants.shouldUsePhysicalBraking;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.GoToPointConstants.tPIDCoeff_GoToPoint;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.GoToPointConstants.tPIDCoeff_finalAdj;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.GoToPointConstants.useFinalAdj;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.GoToPointConstants.velocityThreshold;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.MecanumChassisConstants.Forward;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.MecanumChassisConstants.Heading;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.MecanumChassisConstants.Lateral;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.MecanumChassisConstants.forwardMultiplier;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.MecanumChassisConstants.headingMultiplier;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.MecanumChassisConstants.lateralMultiplier;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.WAIT_TIME_VARIABLE;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.disableWarningErrors;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.robotLengthInCMs;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.robotWidthInCMs;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.useDashboard;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Globals.isAuto;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Chassis.MecanumChassis;
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
import java.util.concurrent.TimeUnit;

public class SpeediDrive implements Module {
    private static final SquidController tPid = new SquidController(tPIDCoeff_GoToPoint.p, tPIDCoeff_GoToPoint.i, tPIDCoeff_GoToPoint.d, 0);
    private static final PIDController hPid = new PIDController(hPIDCoeff_GoToPoint.p, hPIDCoeff_GoToPoint.i, hPIDCoeff_GoToPoint.d);
    public PinPointLocalizer localizer;
    public MecanumChassis motors;
    public boolean robotIsStuck = false;
    public SplineFollower follower;
    public Spline curve = null;
    public DashboardPoseTracker poseTracker;
    Telemetry telemetry;
    HardwareMap hardwareMap;
    Vector powerVector;
    boolean timerResetFailsafe = false, trajectoryDone = true;
    ElapsedTime timer = new ElapsedTime(), failsafeTimer = new ElapsedTime();
    Queue<Pose> targetPositions = new LinkedList<>();
    boolean isOnlyTarget = false;
    ElapsedTime timerSinceStart = new ElapsedTime();
    double startAngleTraj = 0;
    RunMode runMode = RunMode.MANUAL;
    boolean customTolerance = false;
    Pose tolerance = new Pose(), lastTolerance = new Pose(WAIT_TIME_VARIABLE, WAIT_TIME_VARIABLE, DistanceUnit.CM, WAIT_TIME_VARIABLE, AngleUnit.RADIANS);
    double smoothingDist = 0;
    double initialDistance = 0;
    boolean shouldWaitToStop = false, lastShouldWaitToStop = false;
    public boolean noGoZone = false, willEnterNoGoZone = false, goingToNewTargetForAvoidingNoGoZone = false;
    public Pose topLeftCorner = new Pose(), topRightCorner = new Pose(), bottomLeftCorner = new Pose(), bottomRightCorner = new Pose();
    private Pose lastTarget = new Pose();
    private Pose targetPose = new Pose(), falseTargetPose = new Pose();
    private Vector speedVector = new Vector(0, 0, 0);
    private int n;


    public SpeediDrive(HardwareMap hardwareMap, Pose startPose, Telemetry telemetry, boolean isAuto) {
        Globals.isAuto = isAuto;
        motors = new MecanumChassis(hardwareMap, !shouldUsePhysicalBraking);
        localizer = new PinPointLocalizer(hardwareMap, startPose, telemetry);

        if (useDashboard) poseTracker = new DashboardPoseTracker(localizer);
        timer.reset();
        timerSinceStart.reset();
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        tPid.setPIDF(tPIDCoeff_GoToPoint.p, tPIDCoeff_GoToPoint.i, tPIDCoeff_GoToPoint.d, 0);
        hPid.setPIDF(hPIDCoeff_GoToPoint.p, hPIDCoeff_GoToPoint.i, hPIDCoeff_GoToPoint.d, 0);

        tPid.reset();
        hPid.reset();

    }

    public SpeediDrive(HardwareMap hardwareMap, Telemetry telemetry, boolean isAuto) {
        Globals.isAuto = isAuto;
        motors = new MecanumChassis(hardwareMap, !shouldUsePhysicalBraking);
        localizer = new PinPointLocalizer(hardwareMap, new Pose(), telemetry);

        if (useDashboard) poseTracker = new DashboardPoseTracker(localizer);
        timer.reset();
        timerSinceStart.reset();
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        tPid.setPIDF(tPIDCoeff_GoToPoint.p, tPIDCoeff_GoToPoint.i, tPIDCoeff_GoToPoint.d, 0);
        hPid.setPIDF(hPIDCoeff_GoToPoint.p, hPIDCoeff_GoToPoint.i, hPIDCoeff_GoToPoint.d, 0);

        tPid.reset();
        hPid.reset();

    }

    public SpeediDrive(HardwareMap hardwareMap, Pose startPose, Telemetry telemetry, boolean brake, boolean isAuto) {
        Globals.isAuto = isAuto;
        motors = new MecanumChassis(hardwareMap, brake);
        localizer = new PinPointLocalizer(hardwareMap, startPose, telemetry);
        if (useDashboard) poseTracker = new DashboardPoseTracker(localizer);
        timer.reset();
        timerSinceStart.reset();
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        tPid.setPIDF(tPIDCoeff_GoToPoint.p, tPIDCoeff_GoToPoint.i, tPIDCoeff_GoToPoint.d, 0);
        hPid.setPIDF(hPIDCoeff_GoToPoint.p, hPIDCoeff_GoToPoint.i, hPIDCoeff_GoToPoint.d, 0);

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
        follower = new SplineFollower(localizer.getPoseEstimate(), trajectory, telemetry);
        targetPose = new Pose(trajectory.calculate(1), trajectory.heading(1));
        falseTargetPose = targetPose;
        startAngleTraj = localizer.getPoseEstimate().getHeading(AngleUnit.RADIANS);
        trajectoryDone = false;
        robotIsStuck = false;
        timerSinceStart.reset();
        timerResetFailsafe = false;
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
        follower = new SplineFollower(localizer.getPoseEstimate(), trajectory, telemetry, Range.clip(rateOfChange, 0, 1));
        targetPose = new Pose(trajectory.calculate(1), trajectory.heading(1));
        falseTargetPose = targetPose;
        robotIsStuck = false;
        startAngleTraj = localizer.getPoseEstimate().getHeading(AngleUnit.RADIANS);
        trajectoryDone = false;
        timerSinceStart.reset();
        timerResetFailsafe = false;
    }

    public void setSpline_withTangentialHeadingChange(Spline trajectory) {
        this.runMode = RunMode.Spline;
        this.curve = trajectory;
        localizer.update();
        follower = new SplineFollower(localizer.getPoseEstimate(), trajectory, telemetry);
        targetPose = new Pose(trajectory.calculate(1), trajectory.heading(1));
        falseTargetPose = targetPose;
        robotIsStuck = false;
        startAngleTraj = localizer.getPoseEstimate().getHeading(AngleUnit.RADIANS);
        trajectoryDone = false;
        timerSinceStart.reset();
        timerResetFailsafe = false;
    }

    public void setSpeedVector(Vector speedVector) {
        this.runMode = RunMode.MANUAL;
        this.speedVector = speedVector;
    }

    public void setTargetPose(Pose targetPose, boolean shouldWaitToStopCompletelyAtTheEndOfTrajectory) {
        this.shouldWaitToStop = shouldWaitToStopCompletelyAtTheEndOfTrajectory;
        this.runMode = RunMode.PID;
        this.targetPose = targetPose;
        falseTargetPose = targetPose;
        targetPositions.clear();
        trajectoryDone = false;
        isOnlyTarget = true;
        robotIsStuck = false;
        startAngleTraj = localizer.getPoseEstimate().getHeading(AngleUnit.RADIANS);
        timerSinceStart.reset();
        timerResetFailsafe = false;
        motors.setMinPowersToOvercomeFriction();

        tPid.setPIDF(tPIDCoeff_GoToPoint.p, tPIDCoeff_GoToPoint.i, tPIDCoeff_GoToPoint.d, 0);
        hPid.setPIDF(hPIDCoeff_GoToPoint.p, hPIDCoeff_GoToPoint.i, hPIDCoeff_GoToPoint.d, 0);

        tPid.reset();
        hPid.reset();
        initialDistance = targetPose.distanceTo(localizer.getPoseEstimate(), DistanceUnit.CM);

        if (noGoZone && !goingToNewTargetForAvoidingNoGoZone) {
            if (checkIfInsideNoGoZone(localizer.getPoseEstimate(), targetPose)) {
                recalibrateTargetToAvoidNoGoZone();
            }
        }
    }

    public void setTargetPose(Pose targetPose, Pose tolerance, boolean shouldWaitToStopCompletelyAtTheEndOfTrajectory) {
        this.shouldWaitToStop = shouldWaitToStopCompletelyAtTheEndOfTrajectory;
        this.runMode = RunMode.PID;
        this.targetPose = targetPose;
        falseTargetPose = targetPose;
        targetPositions.clear();
        trajectoryDone = false;
        isOnlyTarget = true;
        startAngleTraj = localizer.getPoseEstimate().getHeading(AngleUnit.RADIANS);
        timerSinceStart.reset();
        timerResetFailsafe = false;
        customTolerance = true;
        robotIsStuck = false;
        this.tolerance = tolerance;

        motors.setMinPowersToOvercomeFriction();

        tPid.setPIDF(tPIDCoeff_GoToPoint.p, tPIDCoeff_GoToPoint.i, tPIDCoeff_GoToPoint.d, 0);
        hPid.setPIDF(hPIDCoeff_GoToPoint.p, hPIDCoeff_GoToPoint.i, hPIDCoeff_GoToPoint.d, 0);

        tPid.reset();
        hPid.reset();

        initialDistance = targetPose.distanceTo(localizer.getPoseEstimate(), DistanceUnit.CM);
        if (noGoZone && !goingToNewTargetForAvoidingNoGoZone) {
            if (checkIfInsideNoGoZone(localizer.getPoseEstimate(), targetPose)) {
                recalibrateTargetToAvoidNoGoZone();
            }
        }
    }

    public void updateTargetPose(Pose targetPose, Pose tolerance, boolean shouldWaitToStopCompletelyAtTheEndOfTrajectory) {
        this.shouldWaitToStop = shouldWaitToStopCompletelyAtTheEndOfTrajectory;
        this.runMode = RunMode.PID;
        this.targetPose = targetPose;
        falseTargetPose = targetPose;
        targetPositions.clear();
        trajectoryDone = false;
        isOnlyTarget = true;
        startAngleTraj = localizer.getPoseEstimate().getHeading(AngleUnit.RADIANS);
        customTolerance = true;
        robotIsStuck = false;
        timerSinceStart.reset();
        this.tolerance = tolerance;
        timerResetFailsafe = false;

        initialDistance = targetPose.distanceTo(localizer.getPoseEstimate(), DistanceUnit.CM);
        if (noGoZone && !goingToNewTargetForAvoidingNoGoZone) {
            if (checkIfInsideNoGoZone(localizer.getPoseEstimate(), targetPose)) {
                recalibrateTargetToAvoidNoGoZone();
            }
        }
    }

    public void updateTargetPose(Pose targetPose, boolean shouldWaitToStopCompletelyAtTheEndOfTrajectory) {
        this.shouldWaitToStop = shouldWaitToStopCompletelyAtTheEndOfTrajectory;
        this.runMode = RunMode.PID;
        this.targetPose = targetPose;
        falseTargetPose = targetPose;
        targetPositions.clear();
        trajectoryDone = false;
        isOnlyTarget = true;
        startAngleTraj = localizer.getPoseEstimate().getHeading(AngleUnit.RADIANS);
        customTolerance = false;
        robotIsStuck = false;
        timerSinceStart.reset();
        timerResetFailsafe = false;

        initialDistance = targetPose.distanceTo(localizer.getPoseEstimate(), DistanceUnit.CM);
        if (noGoZone && !goingToNewTargetForAvoidingNoGoZone) {
            if (checkIfInsideNoGoZone(localizer.getPoseEstimate(), targetPose)) {
                recalibrateTargetToAvoidNoGoZone();
            }
        }
    }

    public void setTargetsList(Queue<Pose> targetPositions, double smoothingDistance) {
        this.targetPositions.clear();
        this.smoothingDist = smoothingDistance;
        this.runMode = RunMode.PID;
        this.targetPositions = targetPositions;
        this.shouldWaitToStop = false;
        n = targetPositions.size();
        if (targetPositions.size() > 1) {
            targetPose = targetPositions.poll();
            falseTargetPose = calculateFalseTarget(targetPose);
            startAngleTraj = localizer.getPoseEstimate().getHeading(AngleUnit.RADIANS);
            trajectoryDone = false;
            isOnlyTarget = false;
            robotIsStuck = false;
            timerSinceStart.reset();
            timerResetFailsafe = false;
            initialDistance = targetPose.distanceTo(localizer.getPoseEstimate(), DistanceUnit.CM);
        } else
            setTargetPose(targetPose, false);
    }

    public Pose calculateFalseTarget(Pose target) {
        double cx = localizer.getPoseEstimate().getX(DistanceUnit.CM), cy = localizer.getPoseEstimate().getY(DistanceUnit.CM);
        double tx = target.getX(DistanceUnit.CM), ty = target.getY(DistanceUnit.CM);

        double dx = tx - cx, dy = ty - cy;
        double len = Math.hypot(dx, dy);

        // unit direction from current -> target
        double ux = dx / len, uy = dy / len;

        double fx = tx + Math.signum(ux) * smoothingDist;
        double fy = ty + Math.signum(uy) * smoothingDist;
        return new Pose(fx, fy, DistanceUnit.CM, target.getHeading(AngleUnit.RADIANS), AngleUnit.RADIANS);
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
        setTargetPose(getCurrentPos().add(new Pose(distanceInCmLateral, distanceInCmForward, DistanceUnit.CM, degreesToTurn, AngleUnit.DEGREES).rotateFieldCoordinate(-getCurrentPos().getHeading(AngleUnit.RADIANS))), shouldWaitToStopCompletelyAtTheEndOfTrajectory);
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
                if (isOnlyTarget) {
                    if (!customTolerance) {
                        if (reachedTarget(3) && reachedHeading(3) && stopped()) {
                            trajectoryDone = true;
                            robotIsStuck = false;
                        }
                    } else if (getXError(DistanceUnit.CM) <= tolerance.getX(DistanceUnit.CM) && getYError(DistanceUnit.CM) <= tolerance.getY(DistanceUnit.CM) && reachedHeading(tolerance.getHeading(AngleUnit.RADIANS)) && stopped()) {
                        trajectoryDone = true;
                        robotIsStuck = false;
                    }
                } else {
                    if (getPercentageOfTrajectoryDone() >= 96.0) {
                        trajectoryDone = false;
                        robotIsStuck = false;
                        targetPose = targetPositions.poll();
                        initialDistance = targetPose != null ? targetPose.distanceTo(localizer.getPoseEstimate(), DistanceUnit.CM) : 0;
                        if (targetPositions.isEmpty())
                            isOnlyTarget = true;
                    }
                }

                if (goingToNewTargetForAvoidingNoGoZone && trajectoryDone) {
                    if (!lastTolerance.equals(new Pose(WAIT_TIME_VARIABLE, WAIT_TIME_VARIABLE, DistanceUnit.CM, WAIT_TIME_VARIABLE, AngleUnit.RADIANS)))
                        updateTargetPose(lastTarget, lastTolerance, lastShouldWaitToStop);
                    else updateTargetPose(lastTarget, lastShouldWaitToStop);
                    lastTolerance = new Pose(WAIT_TIME_VARIABLE, WAIT_TIME_VARIABLE, DistanceUnit.CM, WAIT_TIME_VARIABLE, AngleUnit.RADIANS);
                    goingToNewTargetForAvoidingNoGoZone = false;
                    willEnterNoGoZone = false;
                    noGoZone = true;
                }

                if (!trajectoryDone) {
                    checkIfRobotIsStuck();
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
                Vector err = targetPose.subtract(currentPose).toVec();
                if (!isOnlyTarget)
                    err = falseTargetPose.subtract(currentPose).toVec();
                err.setHeading(angleWrapper(err.getHeading()));

                if (reachedTarget(10) && reachedHeading(5) && runMode != RunMode.CalibrateSplinePID) {
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
                        hPid.setPIDF(hPIDCoeff_GoToPoint.p, hPIDCoeff_GoToPoint.i, hPIDCoeff_GoToPoint.d, 0);
                    } else {
                        motors.setMinPowersToOvercomeFriction();
                        tPid.setPIDF(tPIDCoeff_SplineFollower.p, tPIDCoeff_SplineFollower.i, tPIDCoeff_SplineFollower.d, 0);
                        hPid.setPID(hPIDCoeff_GoToPoint.p, hPIDCoeff_GoToPoint.i, hPIDCoeff_GoToPoint.d);
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
        if (timerSinceStart.milliseconds() >= 1000 && localizer.getVelocity().getMagnitude() <= velocityThreshold && localizer.getVelocity().getHeading() <= 2 && !timerResetFailsafe) {
            timerResetFailsafe = true;
            failsafeTimer.reset();
        }
        if (timerResetFailsafe && failsafeTimer.milliseconds() >= 300) {
            timerResetFailsafe = false;
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

    public void noGoZoneTelemetry(boolean updated) {
        telemetry.addData("target positions", targetPositions);
        telemetry.addData("number of targets", n);
        telemetry.addData("will avoid zone", goingToNewTargetForAvoidingNoGoZone);
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

    public double getTimeSinceTrajectoryStart(TimeUnit unit) {
        switch (unit) {
            case MILLISECONDS:
                return timerSinceStart.milliseconds();
            case SECONDS:
                return timerSinceStart.seconds();
            case NANOSECONDS:
                return timerSinceStart.nanoseconds();
            case MINUTES:
                return timerSinceStart.seconds() / 60.0;
            case HOURS:
                return timerSinceStart.seconds() / 3600.0;
            default:
                if (!disableWarningErrors)
                    throw new IllegalArgumentException("Are you serious you need it in " + unit + " \uD83D\uDE02");
                else
                    return 0;
        }
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

    public void drive(Gamepad gamepad) {
        motors.resetMinPowersToOvercomeFriction();

        double forward = -gamepad.left_stick_y * motors.getDriveMultiplier();
        double strafe = gamepad.left_stick_x * motors.getDriveMultiplier();
        double turn = gamepad.right_stick_x * motors.getDriveMultiplier();

        Vector drive = new Vector(strafe, forward, turn);
        if (drive.getMagnitude() <= 0.05) {
            drive.scalarMultiply(0);
        }

        //setMotorPower(new Vector[]{new Vector(drive.getX(), Range.clip(drive.getY() + drive.getHeading(), -1, 1)), new Vector(drive.getX(), Range.clip(drive.getY() - drive.getHeading(), -1, 1))});
        motors.setMotorPowerForced(drive);
    }

    public void driveFieldCentric(Gamepad gamepad, double angle) {
        motors.resetMinPowersToOvercomeFriction();

        double forward = (-gamepad.left_stick_y * motors.getDriveMultiplier());
        double strafe = (gamepad.left_stick_x * motors.getDriveMultiplier());
        double turn = (gamepad.right_stick_x * motors.getDriveMultiplier());

        Vector drive = new Vector(strafe, forward, turn).rotate(angle);
        if (drive.getMagnitude() <= 0.05) {
            drive.scalarMultiply(0);
        }

        //setMotorPower(new Vector[]{new Vector(drive.getX(), Range.clip(drive.getY() + drive.getHeading(), -1, 1)), new Vector(drive.getX(), Range.clip(drive.getY() - drive.getHeading(), -1, 1))});
        motors.setMotorPowerForced(drive);
    }

    public double smoothControls(double value) {
        return 0.5 * Math.tan(1.12 * value);
    }

    public void setNoGoZone(Pose topLeftCorner, Pose bottomRightCorner, double smoothingDistanceInCaseAvoidingNeeded) {
        this.smoothingDist = smoothingDistanceInCaseAvoidingNeeded;
        this.topLeftCorner = topLeftCorner;
        this.topRightCorner = new Pose(bottomRightCorner.getX(DistanceUnit.CM), topLeftCorner.getY(DistanceUnit.CM), DistanceUnit.CM);
        this.bottomRightCorner = bottomRightCorner;
        this.bottomLeftCorner = new Pose(topLeftCorner.getX(DistanceUnit.CM), bottomRightCorner.getY(DistanceUnit.CM), DistanceUnit.CM);
        if (!disableWarningErrors && topLeftCorner.getX(DistanceUnit.CM) >= bottomRightCorner.getX(DistanceUnit.CM) || topLeftCorner.getY(DistanceUnit.CM) <= bottomRightCorner.getY(DistanceUnit.CM))
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

    public boolean checkIfInsideNoGoZone(Pose startPose, Pose targetPose) {
        return segmentIntersectsRect(startPose, targetPose);
    }


    public void recalibrateTargetToAvoidNoGoZone() {
        willEnterNoGoZone = false;
        goingToNewTargetForAvoidingNoGoZone = true;
        lastTarget = targetPose;
        if (customTolerance)
            lastTolerance = tolerance;
        lastShouldWaitToStop = shouldWaitToStop;
        Queue<Pose> targetPoses = findBestCorners(localizer.getPoseEstimate(), targetPose);
        if (targetPoses == null) {
            goingToNewTargetForAvoidingNoGoZone = false;
            willEnterNoGoZone = false;
            noGoZone = true;

            this.shouldWaitToStop = lastShouldWaitToStop;
            this.runMode = RunMode.PID;
            this.targetPose = lastTarget;
            targetPositions.clear();
            trajectoryDone = false;
            isOnlyTarget = true;
            startAngleTraj = localizer.getPoseEstimate().getHeading(AngleUnit.RADIANS);
            customTolerance = true;
            robotIsStuck = false;
            timerSinceStart.reset();
            this.tolerance = lastTolerance;
            timerResetFailsafe = false;
        } else
            setTargetsList(targetPoses, smoothingDist);
    }

    public Queue<Pose> findBestCorners(Pose start, Pose target) {
        double left = topLeftCorner.getX(DistanceUnit.CM);
        double right = bottomRightCorner.getX(DistanceUnit.CM);
        double top = topLeftCorner.getY(DistanceUnit.CM);
        double bottom = bottomRightCorner.getY(DistanceUnit.CM);

        // If start/target fall inside inflated rect, push them to nearest outside point (not a corner)
        Pose s = pointInRect(start, left, right, bottom, top) ? clampToOutside(start) : start;
        Pose t = pointInRect(target, left, right, bottom, top) ? clampToOutside(target) : target;

        Pose[] corners = new Pose[]{
                topLeftCorner, // TL
                topRightCorner, // TR
                bottomRightCorner, // BR
                bottomLeftCorner  // BL
        };
        Pose[] cornersWithTolerance = new Pose[]{
                topLeftCorner.add(new Pose(-robotWidthInCMs / 2, robotLengthInCMs / 2, DistanceUnit.CM)), // TL
                topRightCorner.add(new Pose(robotWidthInCMs / 2, robotLengthInCMs / 2, DistanceUnit.CM)), // TR
                bottomRightCorner.add(new Pose(robotWidthInCMs / 2, -robotLengthInCMs / 2, DistanceUnit.CM)), // BR
                bottomLeftCorner.add(new Pose(-robotWidthInCMs / 2, -robotLengthInCMs / 2, DistanceUnit.CM))  // BL
        };
        Pose bestCorner = new Pose();
        double minDist = Double.MAX_VALUE;
        for (int i = 0; i < 4; i++) {
            Pose c = corners[i];
            if (!segmentIntersectsRect(s, c) && !segmentIntersectsRect(c, t) && isPoseInsideTheField(cornersWithTolerance[i])) {
                if (minDist > c.distanceTo(s, DistanceUnit.CM)) {
                    minDist = c.distanceTo(s, DistanceUnit.CM);
                    bestCorner = cornersWithTolerance[i];
                }
            }
        }
        if (minDist != Double.MAX_VALUE) {
            Queue<Pose> poses = new LinkedList<>();
            poses.add(bestCorner);
            return poses;
        }
        double a = Double.MAX_VALUE;
        double b = Double.MAX_VALUE;
        double c = Double.MAX_VALUE;
        double d = Double.MAX_VALUE;
        if (isPoseInsideTheField(cornersWithTolerance[2]))
            a = bottomRightCorner.distanceTo(targetPose, DistanceUnit.CM);
        if (isPoseInsideTheField(cornersWithTolerance[3]))
            b = bottomLeftCorner.distanceTo(targetPose, DistanceUnit.CM);
        if (isPoseInsideTheField(cornersWithTolerance[0]))
            c = topLeftCorner.distanceTo(targetPose, DistanceUnit.CM);
        if (isPoseInsideTheField(cornersWithTolerance[1]))
            d = topRightCorner.distanceTo(targetPose, DistanceUnit.CM);
        Pose bestCornerWithTolerance;

        minDist = Math.min(Math.min(Math.min(a, b), c), d);
        if (minDist == a && a != Double.MAX_VALUE) {
            bestCornerWithTolerance = cornersWithTolerance[2];
            bestCorner = bottomRightCorner;
        } else if (minDist == b && b != Double.MAX_VALUE) {
            bestCornerWithTolerance = cornersWithTolerance[3];
            bestCorner = bottomLeftCorner;
        } else if (minDist == c && c != Double.MAX_VALUE) {
            bestCornerWithTolerance = cornersWithTolerance[0];
            bestCorner = topLeftCorner;
        } else if (d != Double.MAX_VALUE) {
            bestCornerWithTolerance = cornersWithTolerance[1];
            bestCorner = topRightCorner;
        } else if (!disableWarningErrors) {
            throw new RuntimeException("no go zone can not be avoided");
        } else return null;
        Queue<Pose> bestCorners = new LinkedList<>();

        for (int i = 0; i < 4; i++) {
            Pose corner = corners[i];
            if (bestCorner != corner && areAdjacentAxisAligned(corner, bestCorner) && !segmentIntersectsRect(corner, s) && isPoseInsideTheField(cornersWithTolerance[i])) {
                bestCorners.add(cornersWithTolerance[i]);
                break;
            }
        }
        if (bestCorners.isEmpty()) {
            if (!disableWarningErrors)
                throw new RuntimeException("no go zone can not be avoided");
            else return null;
        }
        bestCorners.add(bestCornerWithTolerance);
        return bestCorners;
    }

    private static final double EPS = 1e-10;

    public boolean areAdjacentAxisAligned(Pose a, Pose b) {
        double ax = a.getX(DistanceUnit.CM), ay = a.getY(DistanceUnit.CM);
        double bx = b.getX(DistanceUnit.CM), by = b.getY(DistanceUnit.CM);

        return (ax == bx) || (ay == by); // exactly one coordinate matches
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

    private boolean pointInRect(Pose p, double left, double right, double bottom, double top) {
        double x = p.getX(DistanceUnit.CM), y = p.getY(DistanceUnit.CM);
        return x > left && x < right && y > bottom && y < top && isPoseInsideTheField(p); // strict interior
    }

    private Pose clampToOutside(Pose a) {
        if (!disableWarningErrors)
            throw new IllegalArgumentException("Target or start is inside no go zone or outside the field");
        else
            return a;
    }

    public Pose getLastTarget() {
        if (goingToNewTargetForAvoidingNoGoZone && willEnterNoGoZone)
            return lastTarget;
        else
            return new Pose(WAIT_TIME_VARIABLE, WAIT_TIME_VARIABLE, DistanceUnit.CM);
    }

    Pose eps = new Pose(0.1, -0.1, DistanceUnit.CM);

    private boolean segmentIntersectsRect(Pose a, Pose b) {
        return segmentsIntersect(a, b, topLeftCorner.subtract(eps), topRightCorner.add(eps)) || // top
                segmentsIntersect(a, b, topRightCorner.add(eps), bottomRightCorner.subtract(eps)) || // right
                segmentsIntersect(a, b, bottomRightCorner.subtract(eps), bottomLeftCorner.add(eps)) || // bottom
                segmentsIntersect(a, b, bottomLeftCorner.add(eps), topLeftCorner.subtract(eps));   // left
    }

    private static boolean segmentsIntersect(Pose p1, Pose q1, Pose p2, Pose q2) {
        int o1 = orientation(p1, q1, p2);
        int o2 = orientation(p1, q1, q2);
        int o3 = orientation(p2, q2, p1);
        int o4 = orientation(p2, q2, q1);

        // Proper intersection: strict opposite orientations
        return o1 * o2 < 0 && o3 * o4 < 0;
    }

    private static int orientation(Pose a, Pose b, Pose c) {
        Vector aVec = new Vector(a);
        Vector bVec = new Vector(b);
        Vector cVec = new Vector(c);

        // cross( b - a, c - a )
        double val = Vector.crossProduct(bVec.subtract(aVec), cVec.subtract(aVec));
        if (Math.abs(val) <= EPS) return 0;
        return val > 0 ? +1 : -1;  // +1: CCW, -1: CW
    }

    public double getPercentageOfTrajectoryDone() {
        if (runMode == RunMode.Spline) {
            return follower.percentageOfTrajectoryThatIsDone();
        } else if (runMode != RunMode.MANUAL) {
            double currentDistToTarget = targetPose.distanceTo(localizer.getPoseEstimate(), DistanceUnit.CM);
            return (100.0 - currentDistToTarget / initialDistance * 100.0);
        }
        return 0;
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

    public boolean isPoseInsideTheField(Pose target) {
        return target.getX(DistanceUnit.CM) >= -366 && target.getX(DistanceUnit.CM) <= 366 &&
                target.getY(DistanceUnit.CM) >= -366 && target.getY(DistanceUnit.CM) <= 366;
    }

    public enum RunMode {
        PID, MANUAL, Spline, CalibrateSplinePID
    }
}