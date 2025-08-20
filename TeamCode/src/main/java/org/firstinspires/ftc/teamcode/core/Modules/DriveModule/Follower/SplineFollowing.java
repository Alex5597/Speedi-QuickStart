package org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Follower;

import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.DriveCorrectionCoefficients.hPIDCoeff;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.DriveCorrectionCoefficients.xPIDCoeff_Spline;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.DriveCorrectionCoefficients.yPIDCoeff_Spline;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.FollowerConstants.MaxForwardAcceleration;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.FollowerConstants.MaxLateralAccelerationWithoutSlippage;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.FollowerConstants.SPEED_SAFETY;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.FollowerConstants.resolution;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.FollowerConstants.speedPIDCoeff_Spline;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.WAIT_TIME_VARIABLE;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.yDeceleration;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.yMaxVelocity;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Localizer.Localizer;
import org.firstinspires.ftc.teamcode.core.Util.Algorithm.SplineGenerator.Spline;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;
import org.firstinspires.ftc.teamcode.core.Util.Math.Vector;
import org.firstinspires.ftc.teamcode.core.Util.utils.Constants;

@Config
public class SplineFollowing {
    public Localizer localizer;
    public Spline trajectory;
    Vector finalPower = new Vector(0, 0);

    public static PIDController speedPid = new PIDController(
            speedPIDCoeff_Spline.p, speedPIDCoeff_Spline.i, speedPIDCoeff_Spline.d);
    public static PIDController xPid = new PIDController(
            xPIDCoeff_Spline.p, xPIDCoeff_Spline.i, xPIDCoeff_Spline.d);
    public static PIDController yPid = new PIDController(
            yPIDCoeff_Spline.p, yPIDCoeff_Spline.i, yPIDCoeff_Spline.d);
    public static PIDController hPid = new PIDController(
            hPIDCoeff.p, hPIDCoeff.i, hPIDCoeff.d);

    Telemetry telemetry;
    double lastT = 0;
    Vector finalPoint;
    boolean goToPoint, instantHeading;
    double tLerp = 0;

    // ---- Speed profile (computed once per trajectory) ----
    private boolean speedReady = false;
    private double[] tS, sS, vLim, vFeas;

    public SplineFollowing(Localizer localizer, @NonNull Spline trajectory, Telemetry telemetry) {
        this.trajectory = trajectory;
        this.localizer = localizer;
        this.telemetry = telemetry;
        this.instantHeading = true;

        finalPoint = trajectory.calculate(1);
        goToPoint = false;
        trajectory.setFirstHeading(localizer.getPoseEstimate().getHeading());
        buildSpeedProfileIfNeeded();

        xPid.setPID(xPIDCoeff_Spline.p, xPIDCoeff_Spline.i, xPIDCoeff_Spline.d);
        yPid.setPID(yPIDCoeff_Spline.p, yPIDCoeff_Spline.i, yPIDCoeff_Spline.d);
        hPid.setPID(hPIDCoeff.p, hPIDCoeff.i, hPIDCoeff.d);

        xPid.reset();
        yPid.reset();
        hPid.reset();
    }

    public SplineFollowing(Localizer localizer, @NonNull Spline trajectory, Telemetry telemetry, double rateOfChange) {
        this.trajectory = trajectory;
        this.localizer = localizer;
        this.telemetry = telemetry;

        this.instantHeading = false;
        tLerp = rateOfChange;

        finalPoint = trajectory.calculate(1);
        goToPoint = false;
        trajectory.setFirstHeading(localizer.getPoseEstimate().getHeading());
        buildSpeedProfileIfNeeded();

        xPid.setPID(xPIDCoeff_Spline.p, xPIDCoeff_Spline.i, xPIDCoeff_Spline.d);
        yPid.setPID(yPIDCoeff_Spline.p, yPIDCoeff_Spline.i, yPIDCoeff_Spline.d);
        hPid.setPID(hPIDCoeff.p, hPIDCoeff.i, hPIDCoeff.d);

        xPid.reset();
        yPid.reset();
        hPid.reset();
    }

    /**
     * Build curvature/accel/ brake-feasible speed profile once per trajectory.
     */
    private void buildSpeedProfileIfNeeded() {
        if (speedReady || trajectory == null) return;

        final int N = Math.max(3, resolution);
        tS = new double[N]; // samples of t in [0,1]
        sS = new double[N]; // cumulative arc length
        vLim = new double[N]; // curvature-limited speeds
        vFeas = new double[N]; // accel/decel-feasible speeds

        Vector prev = trajectory.calculate(0.0);
        tS[0] = 0.0;
        sS[0] = 0.0;
        vLim[0] = curvatureLimit(trajectory.curvatureOfThePath(0.0));

        for (int i = 1; i < N; i++) {
            double t = (double) i / (N - 1);
            tS[i] = t;
            Vector p = trajectory.calculate(t);
            double ds = Math.hypot(p.getX() - prev.getX(), p.getY() - prev.getY());
            sS[i] = sS[i - 1] + ds;
            vLim[i] = curvatureLimit(trajectory.curvatureOfThePath(t));
            prev = p;
        }

        // Initial along-path speed = projection of current velocity on the path tangent at t=0
        double v0 = 0.0;
        try {
            Vector tan0 = trajectory.firstDerivative(0.0).scaleToMagnitude(1);
            v0 = Math.max(0.0, Vector.dot(localizer.getVelocity(), tan0));
        } catch (Exception ignored) { /* keep v0 = 0 */ }

        // Forward pass: respect max forward acceleration
        vFeas[0] = Math.min(vLim[0], v0);
        for (int i = 1; i < N; i++) {
            double ds = Math.max(1e-9, sS[i] - sS[i - 1]);
            double vmaxFromPrev = Math.sqrt(Math.max(0.0,
                    vFeas[i - 1] * vFeas[i - 1] + 2.0 * MaxForwardAcceleration * ds));
            vFeas[i] = Math.min(vLim[i], vmaxFromPrev);
        }

        // Backward pass: ensure we can brake to the next point's speed
        final double aDecel = Math.abs(yDeceleration); // use magnitude
        for (int i = N - 2; i >= 0; i--) {
            double ds = Math.max(1e-9, sS[i + 1] - sS[i]);
            double vBrake = Math.sqrt(Math.max(0.0,
                    vFeas[i + 1] * vFeas[i + 1] + 2.0 * aDecel * ds));
            vFeas[i] = Math.min(vFeas[i], vBrake);
        }

        speedReady = true;
    }

    /**
     * Curvature limit: v <= sqrt(a_lat_max / |kappa|), clamped to yMaxVelocity and safety scaled.
     */
    private double curvatureLimit(double kappa) {
        double k = Math.abs(kappa);
        double vK = (k < 1e-6) ? yMaxVelocity
                : Math.sqrt(Math.max(1e-9, MaxLateralAccelerationWithoutSlippage) / k);
        return Math.min(yMaxVelocity, vK) * SPEED_SAFETY;
    }

    /**
     * Interpolate feasible speed at parameter t ∈ [0,1].
     */
    public double speedAtT(double t) {
        if (vFeas == null || vFeas.length == 0) return 0.0;
        double u = Math.max(0, Math.min(1, t)) * (vFeas.length - 1);
        int i0 = (int) Math.floor(u);
        int i1 = Math.min(vFeas.length - 1, i0 + 1);
        double a = u - i0;
        return vFeas[i0] * (1 - a) + vFeas[i1] * a;
    }

    public Vector getMotorPower() {
        if (goToPoint) return new Vector(WAIT_TIME_VARIABLE, WAIT_TIME_VARIABLE);

        // ---- Find closest point on path ----
        Pose robotPose = localizer.getPoseEstimate();
        double currentT = trajectory.findClosestPoint(robotPose.toVec(), lastT);
        lastT = currentT;

        Vector currTargetPoint = trajectory.calculate(currentT + 1.0 / resolution);
        Pose targetPose = new Pose(currTargetPoint, trajectory.heading(currentT + 1.0 / resolution));
        telemetry.addData("Current t", currentT);

        if (!instantHeading) {
            targetPose.setHeading(hlerp(robotPose.getHeading(), trajectory.heading(currentT), tLerp));
        }

        // ---- Check for final adjustment ----
        if ((currentT >= 0.9 && trajectory.getLength() - trajectory.getLengthAt(currentT)
                <= localizer.getGlideVector().getMagnitude()) || currentT >= 0.95) {
            goToPoint = true;
            telemetry.addLine("GoToPoint activated");
            telemetry.update();
            return new Vector(WAIT_TIME_VARIABLE, WAIT_TIME_VARIABLE);
        }

        // ---- Centripetal correction (anti-slip bias) ----
        double curvature = trajectory.curvatureOfThePath(currentT);
        double vAlongSq = Math.pow(
                Vector.dot(localizer.getVelocity(), trajectory.firstDerivative(currentT).scaleToMagnitude(1)), 2);
        double centripetalTerm = Constants.FollowerConstants.CentripetalScalingFactor
                * Constants.FollowerConstants.TotalMassOfRobot
                * vAlongSq
                * curvature;

        Vector correctionVector = Vector.polar(
                Range.clip(centripetalTerm, -1, 1),
                trajectory.firstDerivative(currentT).getRelativeHeading()
                        + Math.PI / 2 * Math.signum(trajectory.pathNormalVect(currentT).getRelativeHeading())
        );
        telemetry.addData("Centripetal vect", correctionVector.toString());

        // ---- PID correction in robot frame ----
        Pose err = targetPose.subtract(robotPose);
        Vector rotatedErr = err.toVec().rotate(robotPose.getHeading());

        xPid.setPID(xPIDCoeff_Spline.p, xPIDCoeff_Spline.i, xPIDCoeff_Spline.d);
        yPid.setPID(yPIDCoeff_Spline.p, yPIDCoeff_Spline.i, yPIDCoeff_Spline.d);
        hPid.setPID(hPIDCoeff.p, hPIDCoeff.i, hPIDCoeff.d);

        double xPower = xPid.calculate(-rotatedErr.getX(), 0);
        double yPower = yPid.calculate(-rotatedErr.getY(), 0);
        Vector pidVector = new Vector(xPower, yPower);

        if (correctionVector.getMagnitude() > 1)
            correctionVector = correctionVector.scaleToMagnitude(1);
        if (Math.abs(pidVector.getX()) + Math.abs(pidVector.getY()) > 1)
            pidVector = pidVector.scaleToMagnitude(1);

        telemetry.addData("PID power vector", pidVector.toString());
        telemetry.addData("PID x power", xPower);
        telemetry.addData("PID y power", yPower);

        Vector unscaledCorrectionVector = pidVector.add(correctionVector);
        if (unscaledCorrectionVector.getMagnitude() >= 1) {
            double norm = Vector.findScaleFactor(correctionVector, pidVector);
            return correctionVector.add(pidVector.scalarMultiply(norm));
        }

        // ---- Heading correction ----
        double headingDiff = angleWrapper(err.getHeading());
        hPid.setPID(hPIDCoeff.p, hPIDCoeff.i, hPIDCoeff.d);
        double headingPower = -hPid.calculate(-headingDiff, 0);
        unscaledCorrectionVector.setHeading(headingPower);
        telemetry.addData("Heading power", headingPower);

        if (Math.abs(unscaledCorrectionVector.getX())
                + Math.abs(unscaledCorrectionVector.getY())
                + Math.abs(unscaledCorrectionVector.getHeading()) > 1) {
            unscaledCorrectionVector.setHeading(headingPower);
            return unscaledCorrectionVector.scaleToMagnitude_AngularAsWell(1);
        }

        // ---- Path power (track planned speed with predictive look-ahead) ----
        double pathingOrientation = trajectory.firstDerivative(currentT).getRelativeHeading();
        double nextT = trajectory.findClosestPoint(localizer.getPredictedPoseEstimate().toVec(), currentT);

        double idealSpeed = speedAtT(nextT); // target along-path speed from profile
        speedPid.setPID(speedPIDCoeff_Spline.p, speedPIDCoeff_Spline.i, speedPIDCoeff_Spline.d);

        // current along-path speed (projection)
        double vAlong = Vector.dot(localizer.getVelocity(), trajectory.firstDerivative(currentT).scaleToMagnitude(1));
        double power = Range.clip(speedPid.calculate(vAlong, idealSpeed), -1.0, 1.0);

        // keep magnitude = |power| (do NOT normalize to 1)
        Vector pathingPower = Vector.polar(power, pathingOrientation);

        Vector unscaledFinalPower = unscaledCorrectionVector.add(pathingPower);
        telemetry.addData("Pathing power vector", pathingPower.toString());
        telemetry.addData("Pathing power", power);

        if (unscaledFinalPower.getMagnitude() >= 1) {
            double norm = Vector.findScaleFactor(unscaledCorrectionVector, pathingPower);
            return unscaledCorrectionVector.add(pathingPower.scalarMultiply(norm));
        }

        finalPower = unscaledFinalPower;
        telemetry.update();
        return finalPower;
    }

    private double angleWrapper(double angle) {
        if (angle > Math.PI) angle -= 2.0 * Math.PI;
        if (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    private double hlerp(double a, double b, double t) {
        double diff = b - a;
        diff %= 2 * Math.PI;
        if (Math.abs(diff) > Math.PI) {
            if (diff > 0) diff -= 2 * Math.PI;
            else diff += 2 * Math.PI;
        }
        return a + t * diff;
    }
}
