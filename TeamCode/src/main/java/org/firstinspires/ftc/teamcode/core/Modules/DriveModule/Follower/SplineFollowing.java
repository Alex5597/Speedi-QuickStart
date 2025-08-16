package org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Follower;

import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.DriveCorrectionCoefficients.hPIDCoeff;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.DriveCorrectionCoefficients.xPIDCoeff_Spline;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.DriveCorrectionCoefficients.yPIDCoeff_Spline;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.FollowerConstants.A_ACC_MAX;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.FollowerConstants.A_LAT_MAX;
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
import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Drive.Chassis;
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
    public static PIDController speedPid = new PIDController(speedPIDCoeff_Spline.p, speedPIDCoeff_Spline.i, speedPIDCoeff_Spline.d);
    public static PIDController xPid = new PIDController(xPIDCoeff_Spline.p, xPIDCoeff_Spline.i, xPIDCoeff_Spline.d);
    public static PIDController yPid = new PIDController(yPIDCoeff_Spline.p, yPIDCoeff_Spline.i, yPIDCoeff_Spline.d);
    public static PIDController hPid = new PIDController(hPIDCoeff.p, hPIDCoeff.i, hPIDCoeff.d);
    Telemetry telemetry;
    double lastT = 0;
    Vector finalPoint;
    boolean goToPoint, instantHeading;
    double tLerp = 0;

    // ---- Internal arrays ----
    private boolean speedReady = false;
    private double[] tS, sS, vLim, vFeas;

    // Build once per trajectory (call after you set 'trajectory'; set speedReady=false when it changes)
    private void buildSpeedProfileIfNeeded() {
        if (speedReady || trajectory == null) return;
        final int N = Math.max(3, resolution);
        tS = new double[N];
        sS = new double[N];
        vLim = new double[N];
        vFeas = new double[N];

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

        // Forward pass (accel-feasible)
        vFeas[0] = Math.min(vLim[0], 0.0); // start from rest; change if you launch with speed
        for (int i = 1; i < N; i++) {
            double ds = sS[i] - sS[i - 1];
            double vmaxFromPrev = Math.sqrt(Math.max(0.0, vFeas[i - 1] * vFeas[i - 1] + 2 * A_ACC_MAX * ds));
            vFeas[i] = Math.min(vLim[i], vmaxFromPrev);
        }

        // Backward pass (braking-feasible) → "almost braking" before steep curves
        for (int i = N - 2; i >= 0; i--) {
            double ds = sS[i + 1] - sS[i];
            double vBrake = Math.sqrt(Math.max(0.0, vFeas[i + 1] * vFeas[i + 1] + 2 * yDeceleration * ds));
            vFeas[i] = Math.min(vFeas[i], vBrake);
        }

        speedReady = true;
    }

    private double curvatureLimit(double kappa) {
        double k = Math.abs(kappa);
        double vK = (k < 1e-6) ? yMaxVelocity : Math.sqrt(Math.max(1e-9, A_LAT_MAX) / k);
        return Math.min(yMaxVelocity, vK) * SPEED_SAFETY;
    }

    // Linear interpolation of feasible speed at parameter t ∈ [0,1]
    private double speedAtT(double t) {
        if (vFeas == null || vFeas.length == 0) return 0.0;
        double u = Math.max(0, Math.min(1, t)) * (vFeas.length - 1);
        int i0 = (int) Math.floor(u);
        int i1 = Math.min(vFeas.length - 1, i0 + 1);
        double a = u - i0;
        return vFeas[i0] * (1 - a) + vFeas[i1] * a;
    }

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

    public Vector getMotorPower() {
        if (goToPoint)
            return new Vector(WAIT_TIME_VARIABLE, WAIT_TIME_VARIABLE);

        //Finding closest point
        Pose robotPose = localizer.getPoseEstimate();
        double currentT = trajectory.findClosestPoint(robotPose.toVec(), lastT);
        // double currentT = trajectory.findClosestPoint(robotPose.toVec());//TODO try again
        lastT = currentT;
        Vector currTargetPoint = trajectory.calculate(currentT + 1.0 / resolution);
        Pose targetPose = new Pose(currTargetPoint, trajectory.heading(currentT + 1.0 / resolution));
        if (!instantHeading)
            targetPose.setHeading(hlerp(robotPose.getHeading(), trajectory.heading(currentT), tLerp));
        //

        //Check for final adjustment
        if ((currentT >= 0.9 && trajectory.getLength() - trajectory.getLengthAt(currentT) <= localizer.getGlideVector().getMagnitude()) || currentT >= 0.95) {
            goToPoint = true;
            telemetry.addLine("GoToPoint activated");
            telemetry.update();
            return new Vector(WAIT_TIME_VARIABLE, WAIT_TIME_VARIABLE);
        }
        //

        //centripetal Correction
        double curvature = trajectory.curvatureOfThePath(currentT);
        Vector correctionVector = Vector.polar(Range.clip(Constants.FollowerConstants.CentripetalScalingFactor * Constants.FollowerConstants.TotalMassOfRobot * Math.pow(Vector.dot(localizer.getVelocity(), trajectory.firstDerivative(currentT).scaleToMagnitude(1)), 2) * curvature, -1, 1), trajectory.firstDerivative(currentT).getRelativeHeading() + Math.PI / 2 * Math.signum(trajectory.pathNormalVect(currentT).getRelativeHeading()));
        telemetry.addData("Centripetal vect", correctionVector.toString());
        telemetry.update();
        //

        //PID Correction
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
        Vector unscaledCorrectionVector = pidVector.add(correctionVector);
        if (unscaledCorrectionVector.getMagnitude() >= 1) {
            double norm = Vector.findScaleFactor(correctionVector, pidVector);
            return correctionVector.add(pidVector.scalarMultiply(norm));
        }
        //

        //Heading Correction
        double headingDiff = angleWrapper(err.getHeading());
        hPid.setPID(Constants.DriveCorrectionCoefficients.hPIDCoeff.p, Constants.DriveCorrectionCoefficients.hPIDCoeff.i, Constants.DriveCorrectionCoefficients.hPIDCoeff.d);
        double headingPower = -hPid.calculate(-headingDiff, 0);
        unscaledCorrectionVector.setHeading(headingPower);
        if (Math.abs(unscaledCorrectionVector.getX()) + Math.abs(unscaledCorrectionVector.getY()) + Math.abs(unscaledCorrectionVector.getHeading()) > 1) {
            unscaledCorrectionVector.setHeading(headingPower);
            return unscaledCorrectionVector.scaleToMagnitude_AngularAsWell(1);
        }
        //

        //Path Power
        double pathingOrientation = trajectory.firstDerivative(currentT).getRelativeHeading();
        double nextT = trajectory.findClosestPoint(localizer.getPredictedPoseEstimate().toVec(), currentT);
        double idealSpeed = speedAtT(nextT);                 // target along-path speed
        double power = speedPid.calculate(localizer.getVelocity().getMagnitude(), idealSpeed);
        Vector pathingPower = Vector.polar(power, pathingOrientation);

        Vector unscaledFinalPower = unscaledCorrectionVector.add(pathingPower);
        if (unscaledFinalPower.getMagnitude() >= 1) {
            double norm = Vector.findScaleFactor(unscaledCorrectionVector, pathingPower);
            return unscaledCorrectionVector.add(pathingPower.scalarMultiply(norm));
        }
        finalPower = unscaledFinalPower;
        //

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
            if (diff > 0) {
                diff -= 2 * Math.PI;
            } else {
                diff += 2 * Math.PI;
            }
        }
        return a + t * diff;
    }
}