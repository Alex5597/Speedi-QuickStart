package org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Follower;

import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LQRSplineConstants.alongKPosition;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LQRSplineConstants.alongKVelocity;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LQRSplineConstants.alongLagTolerance;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LQRSplineConstants.angularVelocityTolerance;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LQRSplineConstants.crossKPosition;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LQRSplineConstants.crossKVelocity;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LQRSplineConstants.forwardKA;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LQRSplineConstants.forwardKS;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LQRSplineConstants.forwardKV;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LQRSplineConstants.frictionOmegaDeadband;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LQRSplineConstants.frictionVelocityDeadband;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LQRSplineConstants.hPIDCoeff_LQR;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LQRSplineConstants.headingKA;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LQRSplineConstants.headingKPosition;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LQRSplineConstants.headingKS;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LQRSplineConstants.headingKVelocity;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LQRSplineConstants.headingKV;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LQRSplineConstants.headingTolerance;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LQRSplineConstants.maxAcceleration;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LQRSplineConstants.maxCorrectionPower;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LQRSplineConstants.maxDeceleration;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LQRSplineConstants.maxHeadingPower;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LQRSplineConstants.minPathSpeedScale;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LQRSplineConstants.positionTolerance;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LQRSplineConstants.profilePowerBudget;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LQRSplineConstants.slowdownDemandThreshold;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LQRSplineConstants.strafeKA;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LQRSplineConstants.strafeKS;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LQRSplineConstants.strafeKV;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LQRSplineConstants.velocityTolerance;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LQRSplineConstants.xyPIDCoeff_LQR;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.core.Util.Algorithm.PathMotionProfile;
import org.firstinspires.ftc.teamcode.core.Util.Algorithm.SplineGenerator.Spline;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;
import org.firstinspires.ftc.teamcode.core.Util.Math.Vector;

public class LQRSplineFollower {
    private final Spline trajectory;
    private final PathMotionProfile profile;
    private final ElapsedTime timer = new ElapsedTime();
    private final Telemetry telemetry;
    //small trim on top of the LQR for residual errors, mostly through the I term
    private final PIDController xPid = new PIDController(xyPIDCoeff_LQR.p, xyPIDCoeff_LQR.i, xyPIDCoeff_LQR.d);
    private final PIDController yPid = new PIDController(xyPIDCoeff_LQR.p, xyPIDCoeff_LQR.i, xyPIDCoeff_LQR.d);
    private final PIDController hPid = new PIDController(hPIDCoeff_LQR.p, hPIDCoeff_LQR.i, hPIDCoeff_LQR.d);

    private final double profileMaxVelocity;
    private double lastTime = 0;
    private double profileTime = 0;
    private double lastDesiredOmega = 0;
    private double currentT = 0;
    private double currentS = 0;
    private double lastAlongError = 0;
    private double lastCrossError = 0;
    private double lastHeadingError = 0;
    private double lastAlongVelocityError = 0;
    private double lastCrossVelocityError = 0;
    private double lastHeadingVelocityError = 0;
    private double lastPathScale = 1;
    private boolean profileDone = false;
    private boolean started = false;
    private Pose targetPose = new Pose();

    public LQRSplineFollower(Spline trajectory, Telemetry telemetry) {
        this.trajectory = trajectory;
        this.telemetry = telemetry;
        profileMaxVelocity = pathMaxVelocity(trajectory);
        profile = new PathMotionProfile(trajectory.getLength(), profileMaxVelocity, maxAcceleration, maxDeceleration);
        xPid.reset();
        yPid.reset();
        hPid.reset();
        timer.reset();
    }

    /**
     * @return the power vector in ROBOT frame (x = strafe, y = forward, heading = turn), ready for the chassis
     */
    public Vector getMotorPower(Pose robotPose, Vector robotVelocity) {
        if (!started) {
            timer.reset();
            lastTime = 0;
            profileTime = 0;
            lastDesiredOmega = 0;
            started = true;
        }
        xPid.setPIDF(xyPIDCoeff_LQR.p, xyPIDCoeff_LQR.i, xyPIDCoeff_LQR.d, 0);
        yPid.setPIDF(xyPIDCoeff_LQR.p, xyPIDCoeff_LQR.i, xyPIDCoeff_LQR.d, 0);
        hPid.setPIDF(hPIDCoeff_LQR.p, hPIDCoeff_LQR.i, hPIDCoeff_LQR.d, 0);

        double now = timer.seconds();
        double dt = Math.max(now - lastTime, 1e-3);
        lastTime = now;

        // The profile is paused while the robot falls behind the moving target (bump, slip, wall)
        // so the reference can't run away; it resumes as the along-track error shrinks
        double lag = Math.max(0, lastAlongError);
        double lagScale = Range.clip(2.0 - lag / Math.max(alongLagTolerance, 1e-6), 0, 1);
        profileTime += dt * lagScale;

        PathMotionProfile.State state = profile.get(profileTime);
        profileDone = state.done;
        currentS = state.position;
        currentT = trajectory.getTAtLength(currentS);

        Vector targetPoint = trajectory.calculate(currentT);
        Vector tangent = Vector.polar(1, trajectory.firstDerivative(currentT).getRelativeHeading());
        Vector normal = new Vector(-tangent.getY(), tangent.getX());
        double targetHeading = trajectory.heading(currentT);
        double desiredOmega = desiredOmega(currentS, state.velocity);
        double desiredAlpha = (desiredOmega - lastDesiredOmega) / dt;
        lastDesiredOmega = desiredOmega;

        Vector positionError = targetPoint.subtract(robotPose.toVec());
        lastAlongError = Vector.dot(positionError, tangent);
        lastCrossError = Vector.dot(positionError, normal);

        double actualAlongVelocity = Vector.dot(robotVelocity, tangent);
        double actualCrossVelocity = Vector.dot(robotVelocity, normal);
        lastAlongVelocityError = state.velocity - actualAlongVelocity;
        lastCrossVelocityError = -actualCrossVelocity;
        lastHeadingError = Vector.wrapAngle(targetHeading - robotPose.getHeading(AngleUnit.RADIANS));
        lastHeadingVelocityError = desiredOmega - robotVelocity.getHeading();

        double heading = robotPose.getHeading(AngleUnit.RADIANS);
        double curvature = trajectory.curvatureOfThePath(currentT);
        Vector desiredVelocityRobot = tangent.scalarMultiply(state.velocity).rotate(heading);
        Vector desiredAccelerationRobot = tangent.scalarMultiply(state.acceleration)
                .add(normal.scalarMultiply(curvature * state.velocity * state.velocity))
                .rotate(heading);
        Vector feedforward = new Vector(
                strafeKS * smoothSign(desiredVelocityRobot.getX(), frictionVelocityDeadband) + strafeKV * desiredVelocityRobot.getX() + strafeKA * desiredAccelerationRobot.getX(),
                forwardKS * smoothSign(desiredVelocityRobot.getY(), frictionVelocityDeadband) + forwardKV * desiredVelocityRobot.getY() + forwardKA * desiredAccelerationRobot.getY());

        //LQR state feedback in path coordinates + the PID trim on the raw field/heading errors
        double alongPower = alongKPosition * lastAlongError + alongKVelocity * lastAlongVelocityError;
        double crossPower = crossKPosition * lastCrossError + crossKVelocity * lastCrossVelocityError;
        double headingPower = headingKS * smoothSign(desiredOmega, frictionOmegaDeadband) + headingKV * desiredOmega + headingKA * desiredAlpha
                + headingKPosition * lastHeadingError + headingKVelocity * lastHeadingVelocityError
                + hPid.calculate(-lastHeadingError, 0);
        Vector pidCorrectionField = new Vector(
                Range.clip(xPid.calculate(-positionError.getX(), 0), -maxCorrectionPower, maxCorrectionPower),
                Range.clip(yPid.calculate(-positionError.getY(), 0), -maxCorrectionPower, maxCorrectionPower));

        alongPower = Range.clip(alongPower, -maxCorrectionPower, maxCorrectionPower);
        crossPower = Range.clip(crossPower, -maxCorrectionPower, maxCorrectionPower);
        headingPower = Range.clip(headingPower, -maxHeadingPower, maxHeadingPower);

        // Full speed while corrections stay under the threshold; only a significant error slows the path down
        double correctionDemand = Math.max(Math.abs(crossPower) / Math.max(maxCorrectionPower, 1e-6), Math.abs(headingPower) / Math.max(maxHeadingPower, 1e-6));
        double demandOverThreshold = Range.clip((co0prrectionDemand - slowdownDemandThreshold) / Math.max(1.0 - slowdownDemandThreshold, 1e-6), 0, 1);
        double pathScale = 1.0 - demandOverThreshold * (1.0 - minPathSpeedScale);
        lastPathScale = pathScale;
        alongPower = Range.clip(alongPower * pathScale, -maxCorrectionPower, maxCorrectionPower);

        Vector correctionRobot = tangent.scalarMultiply(alongPower).add(normal.scalarMultiply(crossPower)).add(pidCorrectionField).rotate(heading);
        Vector robotPower = feedforward.scalarMultiply(pathScale).add(correctionRobot);
        targetPose = new Pose(targetPoint, targetHeading);
        return new Vector(robotPower.getX(), robotPower.getY(), headingPower).scaleToMagnitude_AngularAsWell(1);
    }

    public boolean isFinished(Pose robotPose, Vector robotVelocity) {
        return profileDone &&
                Math.hypot(lastAlongError, lastCrossError) <= positionTolerance &&
                Math.abs(Math.toDegrees(lastHeadingError)) <= headingTolerance &&
                robotVelocity.getMagnitude() <= velocityTolerance &&
                Math.abs(Math.toDegrees(robotVelocity.getHeading())) <= angularVelocityTolerance;
    }

    public double percentageOfTrajectoryThatIsDone() {
        if (trajectory.getLength() <= 1e-6) return 100;
        return Range.clip(currentS / trajectory.getLength() * 100.0, 0, 100);
    }

    public void telemetry(boolean updated) {
        telemetry.addData("LQR t", currentT);
        telemetry.addData("LQR s", currentS);
        telemetry.addData("LQR profile max velocity", profileMaxVelocity);
        telemetry.addData("LQR path speed scale", lastPathScale);
        telemetry.addData("LQR target", targetPose);
        telemetry.addData("LQR along error", lastAlongError);
        telemetry.addData("LQR cross error", lastCrossError);
        telemetry.addData("LQR heading error", Math.toDegrees(lastHeadingError));
        telemetry.addData("LQR along velocity error", lastAlongVelocityError);
        telemetry.addData("LQR cross velocity error", lastCrossVelocityError);
        telemetry.addData("LQR heading velocity error", Math.toDegrees(lastHeadingVelocityError));
        if (!updated) telemetry.update();
    }

    /**
     * The fastest speed the drive model allows on this path using profilePowerBudget of the available
     * power. Driving forward is faster than strafing, so the limit depends on the angle between the
     * path direction and the robot heading at every point; the smallest limit along the path is used.
     */
    private double pathMaxVelocity(Spline trajectory) {
        double forwardMax = Range.clip((profilePowerBudget - forwardKS) / Math.max(forwardKV, 1e-9), 20, 1000);
        double strafeMax = Range.clip((profilePowerBudget - strafeKS) / Math.max(strafeKV, 1e-9), 20, 1000);

        double pathMax = forwardMax;
        for (int i = 0; i <= 50; i++) {
            double t = i / 50.0;
            double travelAngle = trajectory.firstDerivative(t).getRelativeHeading() + trajectory.heading(t);
            double forwardShare = Math.sin(travelAngle);
            double strafeShare = Math.cos(travelAngle);
            double limit = 1.0 / Math.sqrt(forwardShare * forwardShare / (forwardMax * forwardMax) + strafeShare * strafeShare / (strafeMax * strafeMax));
            pathMax = Math.min(pathMax, limit);
        }
        return pathMax;
    }

    private double desiredOmega(double s, double velocity) {
        double ds = Math.max(1.0, velocity * 0.02);
        double length = trajectory.getLength();
        double s0 = Range.clip(s - ds, 0, length);
        double s1 = Range.clip(s + ds, 0, length);
        if (Math.abs(s1 - s0) <= 1e-6) return 0;
        double h0 = trajectory.heading(trajectory.getTAtLength(s0));
        double h1 = trajectory.heading(trajectory.getTAtLength(s1));
        return Vector.wrapAngle(h1 - h0) / (s1 - s0) * velocity;
    }

    // Linear ramp of the static friction feedforward around zero velocity so its sign can't chatter
    private double smoothSign(double value, double deadband) {
        if (deadband <= 1e-9) return Math.signum(value);
        return Range.clip(value / deadband, -1, 1);
    }
}
