package org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Follower;

import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.DriveCorrectionCoefficients.hPIDCoeff;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.DriveCorrectionCoefficients.xPIDCoeff_Spline;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.DriveCorrectionCoefficients.yPIDCoeff_Spline;
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
    public Localizer localizer;
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

    public SplineFollowing(Localizer localizer, Chassis motors, @NonNull Spline trajectory, Telemetry telemetry) {
        this.trajectory = trajectory;
        this.motors = motors;
        this.localizer = localizer;
        this.telemetry = telemetry;
        this.instantHeading = true;

        finalPoint = trajectory.calculate(1);
        goToPoint = false;
        trajectory.setFirstHeading(localizer.getPoseEstimate().getHeading());

        xPid.setPID(xPIDCoeff_Spline.p, xPIDCoeff_Spline.i, xPIDCoeff_Spline.d);
        yPid.setPID(yPIDCoeff_Spline.p, yPIDCoeff_Spline.i, yPIDCoeff_Spline.d);
        hPid.setPID(hPIDCoeff.p, hPIDCoeff.i, hPIDCoeff.d);

        xPid.reset();
        yPid.reset();
        hPid.reset();
    }

    public SplineFollowing(Localizer localizer, Chassis motors, @NonNull Spline trajectory, Telemetry telemetry, double rateOfChange) {
        this.trajectory = trajectory;
        this.motors = motors;
        this.localizer = localizer;
        this.telemetry = telemetry;

        this.instantHeading = false;
        tLerp = rateOfChange;

        finalPoint = trajectory.calculate(1);
        goToPoint = false;
        trajectory.setFirstHeading(localizer.getPoseEstimate().getHeading());

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
        Vector currTargetPoint = trajectory.calculate(currentT);
        Pose targetPose = new Pose(currTargetPoint, trajectory.heading(currentT));
        if (!instantHeading)
            targetPose.setHeading(hlerp(robotPose.getHeading(), trajectory.heading(currentT), tLerp));
        //

        //Check for final adjustment
        if (currentT >= 0.85 && trajectory.getLength() - trajectory.getLengthAt(currentT) <= localizer.getGlideVector().getMagnitude()) {
            goToPoint = true;
            telemetry.addLine("GoToPoint activated");
            telemetry.update();
            return new Vector(WAIT_TIME_VARIABLE, WAIT_TIME_VARIABLE);
        }
        //

        //centripetal Correction
        double curvature = trajectory.curvatureOfThePath(currentT);
        Vector correctionVector = Vector.polar(Range.clip(Constants.FollowerConstants.CentripetalScalingFactor * Constants.FollowerConstants.TotalMassOfRobot * Math.pow(trajectory.firstDerivative(currentT).scaleToMagnitude(1).getMagnitude(), 2) * curvature, -0.7, 0.7), trajectory.firstDerivative(currentT).getRelativeHeading() + Math.PI / 2 * Math.signum(trajectory.pathNormalVect(currentT).getRelativeHeading()));
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
            double norm = Vector.findScaleFactor(pidVector, correctionVector);
            return pidVector.add(correctionVector.scalarMultiply(norm));
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
        Vector pathingPower = trajectory.firstDerivative(currentT).scaleToMagnitude(1).rotate(robotPose.getHeading());

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