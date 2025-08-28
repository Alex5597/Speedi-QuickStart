package org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Follower;

import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.DriveCorrectionCoefficients.hPIDCoeff;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.DriveCorrectionCoefficients.xPIDCoeff_Spline;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.DriveCorrectionCoefficients.yPIDCoeff_Spline;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.FollowerConstants.resolution;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.WAIT_TIME_VARIABLE;

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
    Chassis motors;
    public Spline trajectory;
    Vector finalPower = new Vector(0, 0);
    public static PIDController xPid = new PIDController(xPIDCoeff_Spline.p, xPIDCoeff_Spline.i, xPIDCoeff_Spline.d);
    public static PIDController yPid = new PIDController(yPIDCoeff_Spline.p, yPIDCoeff_Spline.i, yPIDCoeff_Spline.d);
    public static PIDController hPid = new PIDController(hPIDCoeff.p, hPIDCoeff.i, hPIDCoeff.d);
    Telemetry telemetry;
    double lastT = 0;
    Vector finalPoint;
    boolean goToPoint, instantHeading;
    double tLerp = 0;
    boolean tangential = false;

    public SplineFollowing(Pose startPose, @NonNull Spline trajectory, Telemetry telemetry) {
        this.trajectory = trajectory;
        this.telemetry = telemetry;
        this.instantHeading = true;

        telemetry.addLine(startPose.toString());
        finalPoint = trajectory.calculate(1);
        goToPoint = false;
        trajectory.setFirstHeading(startPose.getHeading());

        xPid.setPID(xPIDCoeff_Spline.p, xPIDCoeff_Spline.i, xPIDCoeff_Spline.d);
        yPid.setPID(yPIDCoeff_Spline.p, yPIDCoeff_Spline.i, yPIDCoeff_Spline.d);
        hPid.setPID(hPIDCoeff.p, hPIDCoeff.i, hPIDCoeff.d);

        xPid.reset();
        yPid.reset();
        hPid.reset();
    }

    public SplineFollowing(Pose startPose, @NonNull Spline trajectory, Telemetry telemetry, double rateOfChange) {
        this.trajectory = trajectory;
        this.telemetry = telemetry;

        this.instantHeading = false;
        tLerp = rateOfChange;

        telemetry.addLine(startPose.toString());
        finalPoint = trajectory.calculate(1);
        goToPoint = false;
        trajectory.setFirstHeading(startPose.getHeading());

        xPid.setPID(xPIDCoeff_Spline.p, xPIDCoeff_Spline.i, xPIDCoeff_Spline.d);
        yPid.setPID(yPIDCoeff_Spline.p, yPIDCoeff_Spline.i, yPIDCoeff_Spline.d);
        hPid.setPID(hPIDCoeff.p, hPIDCoeff.i, hPIDCoeff.d);

        xPid.reset();
        yPid.reset();
        hPid.reset();
    }

    public SplineFollowing(@NonNull Spline trajectory, Telemetry telemetry, boolean tangential) {
        this.trajectory = trajectory;
        this.telemetry = telemetry;

        this.instantHeading = false;
        this.tangential = tangential;

        finalPoint = trajectory.calculate(1);
        goToPoint = false;

        xPid.setPID(xPIDCoeff_Spline.p, xPIDCoeff_Spline.i, xPIDCoeff_Spline.d);
        yPid.setPID(yPIDCoeff_Spline.p, yPIDCoeff_Spline.i, yPIDCoeff_Spline.d);
        hPid.setPID(hPIDCoeff.p, hPIDCoeff.i, hPIDCoeff.d);

        xPid.reset();
        yPid.reset();
        hPid.reset();
    }

    public Vector getMotorPower(Pose robotPose, Vector glideVector) {
        if (goToPoint)
            return new Vector(WAIT_TIME_VARIABLE, WAIT_TIME_VARIABLE);

        // Finding closest point
        double currentT = trajectory.findClosestPoint(robotPose.toVec(), lastT) + 1.0 / resolution;
        lastT = currentT;
        Vector currTargetPoint = trajectory.calculate(currentT);
        Pose targetPose = new Pose(currTargetPoint, trajectory.heading(currentT));
        if (tangential)
            targetPose.setHeading(trajectory.heading(currentT));
        else if (!instantHeading)
            targetPose.setHeading(hlerp(robotPose.getHeading(), trajectory.heading(currentT), tLerp));

        // Check for final adjustment
        if (currentT >= 0.95 && trajectory.getLength() - trajectory.getLengthAt(currentT) <= glideVector.getMagnitude()) {
            goToPoint = true;
            telemetry.addLine("GoToPoint activated");
            telemetry.update();
            return new Vector(WAIT_TIME_VARIABLE, WAIT_TIME_VARIABLE);
        }

        // ---------- Centripetal Correction (robot frame) ----------
        double curvature = trajectory.curvatureOfThePath(currentT);
        double tanField = trajectory.firstDerivative(currentT).getRelativeHeading();
        double tanRobot = angleWrapper(tanField - robotPose.getHeading());
        double normalRobot = angleWrapper(tanRobot + (curvature >= 0 ? Math.PI / 2.0 : -Math.PI / 2.0));

        double corrMag = Range.clip(
                Constants.FollowerConstants.CentripetalScalingFactor
                        * Constants.FollowerConstants.TotalMassOfRobot
                        * Math.abs(curvature),
                -1, 1
        );

        Vector correctionVector = Vector.polar(corrMag, normalRobot);
        if (correctionVector.getMagnitude() >= 1) {
            return correctionVector.scaleToMagnitude(1);
        }
        telemetry.addData("Centripetal vect (robot)", correctionVector.toString());
        telemetry.update();

        // ---------- PID Correction (robot frame) ----------
        Pose err = targetPose.subtract(robotPose);
        Vector rotatedErr = err.toVec().rotate(robotPose.getHeading());

        xPid.setPID(xPIDCoeff_Spline.p, xPIDCoeff_Spline.i, xPIDCoeff_Spline.d);
        yPid.setPID(yPIDCoeff_Spline.p, yPIDCoeff_Spline.i, yPIDCoeff_Spline.d);
        hPid.setPID(hPIDCoeff.p, hPIDCoeff.i, hPIDCoeff.d);

        double xPower = xPid.calculate(-rotatedErr.getX(), 0);
        double yPower = yPid.calculate(-rotatedErr.getY(), 0);
        Vector pidVector = new Vector(xPower, yPower).scaleToMagnitude(1);
        Vector unscaledCorrectionVector = pidVector.add(correctionVector);
        if (unscaledCorrectionVector.getMagnitude() >= 1) {
            double norm = Vector.findScaleFactor(pidVector, correctionVector);
            return pidVector.add(correctionVector.scalarMultiply(norm));
        }

        // ---------- Heading Correction ----------
        double headingDiff = angleWrapper(err.getHeading());
        hPid.setPID(Constants.DriveCorrectionCoefficients.hPIDCoeff.p, Constants.DriveCorrectionCoefficients.hPIDCoeff.i, Constants.DriveCorrectionCoefficients.hPIDCoeff.d);
        double headingPower = -hPid.calculate(-headingDiff, 0);
        unscaledCorrectionVector.setHeading(headingPower);
        if (Math.abs(unscaledCorrectionVector.getX()) + Math.abs(unscaledCorrectionVector.getY()) + Math.abs(unscaledCorrectionVector.getHeading()) > 1) {
            return unscaledCorrectionVector.scaleToMagnitude_AngularAsWell(1);
        }

        // ---------- Path Feedforward (robot frame) ----------
        Vector pathingPower = Vector.polar(1, tanRobot);
        Vector unscaledFinalPower = unscaledCorrectionVector.add(pathingPower);
        if (unscaledFinalPower.getMagnitude() >= 1) {
            double norm = Vector.findScaleFactor(unscaledCorrectionVector, pathingPower);
            return unscaledCorrectionVector.add(pathingPower.scalarMultiply(norm));
        }
        finalPower = unscaledFinalPower;

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
