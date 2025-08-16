package org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Follower;

import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.FollowerConstants.CorrectionMagnitude;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.FollowerConstants.HeadingMagnitude;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.FollowerConstants.PathingMagnitude;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.WAIT_TIME_VARIABLE;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.forwardMultiplier;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.headingMultiplier;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.lateralMultiplier;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Localizer.Localizer;
import org.firstinspires.ftc.teamcode.core.Util.Algorithm.SplineGenerator.Spline;
import org.firstinspires.ftc.teamcode.core.Util.utils.Constants;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;
import org.firstinspires.ftc.teamcode.core.Util.Math.Vector;

@Config
public class SplineFollowerVechi {
    Localizer localizer;
    Spline trajectory;
    Vector finalPower = new Vector(0, 0);
    public final PIDController tpid = new PIDController(0, 0, 0), hpid = new PIDController(0, 0, 0);
    Telemetry telemetry;
    double lastT = 0;
    Vector finalPoint;
    boolean goToPoint, instantHeading;
    double tLerp = 0;
    public SplineFollowerVechi(Localizer localizer, Spline trajectory, Telemetry telemetry) {
        this.trajectory = trajectory;
        this.localizer = localizer;
        this.telemetry = telemetry;
        finalPoint = trajectory.calculate(1);
        goToPoint = false;
        trajectory.setFirstHeading(localizer.getPoseEstimate().getHeading());

        tpid.setPID(Constants.DriveCorrectionCoefficients.tPIDCoeff_GoToPoint.p, Constants.DriveCorrectionCoefficients.tPIDCoeff_GoToPoint.i, Constants.DriveCorrectionCoefficients.tPIDCoeff_GoToPoint.d);
        hpid.setPID(Constants.DriveCorrectionCoefficients.hPIDCoeff.p, Constants.DriveCorrectionCoefficients.hPIDCoeff.i, Constants.DriveCorrectionCoefficients.hPIDCoeff.d);
        tpid.reset();
        hpid.reset();
    }
    public SplineFollowerVechi(Localizer localizer, @NonNull Spline trajectory, Telemetry telemetry, double rateOfChange) {
        this.trajectory = trajectory;
        this.localizer = localizer;
        this.telemetry = telemetry;

        this.instantHeading = false;
        tLerp = rateOfChange;

        finalPoint = trajectory.calculate(1);
        goToPoint = false;
        trajectory.setFirstHeading(localizer.getPoseEstimate().getHeading());

        tpid.setPID(Constants.DriveCorrectionCoefficients.tPIDCoeff_GoToPoint.p, Constants.DriveCorrectionCoefficients.tPIDCoeff_GoToPoint.i, Constants.DriveCorrectionCoefficients.tPIDCoeff_GoToPoint.d);
        hpid.setPID(Constants.DriveCorrectionCoefficients.hPIDCoeff.p, Constants.DriveCorrectionCoefficients.hPIDCoeff.i, Constants.DriveCorrectionCoefficients.hPIDCoeff.d);
        tpid.reset();
        hpid.reset();
    }
    public Vector getMotorPower() {
        if (goToPoint)
            return new Vector(WAIT_TIME_VARIABLE, WAIT_TIME_VARIABLE);

        //Finding closest point
        Pose robotPose = localizer.getPoseEstimate();
        double currentT = trajectory.findClosestPoint(robotPose.toVec(), lastT);
        Vector currTargetPoint = trajectory.calculate(currentT);
        Pose targetPose = new Pose(currTargetPoint, trajectory.heading(currentT));
        if (!instantHeading)
            targetPose.setHeading(hlerp(robotPose.getHeading(), trajectory.heading(currentT), tLerp));
        lastT = currentT;
        //

        //Check for final adjustment
        if (trajectory.getLength() - trajectory.getLengthAt(currentT) <= localizer.getGlideVector().getMagnitude()) {
            goToPoint = true;
            telemetry.addLine("GoToPoint activated");
            telemetry.update();
            return new Vector(WAIT_TIME_VARIABLE, WAIT_TIME_VARIABLE);
        }
        //


        //Path Power
        double driveAngle = trajectory.firstDerivative(currentT).getRelativeHeading();
        Vector pathingPower = new Vector(Math.cos(driveAngle), Math.sin(driveAngle)).scalarMultiply(PathingMagnitude);
        //


        //centripetal Correction
        double curvature = trajectory.curvatureOfThePath(currentT);
        Vector correctionVector = Vector.polar(Range.clip(Constants.FollowerConstants.CentripetalScalingFactor * Constants.FollowerConstants.TotalMassOfRobot * Math.pow(Vector.dot(localizer.getVelocity(), trajectory.firstDerivative(currentT).scaleToMagnitude(1)), 2) * curvature, -1, 1), trajectory.firstDerivative(currentT).getRelativeHeading() + Math.PI / 2 * Math.signum(trajectory.pathNormalVect(currentT).getRelativeHeading()));
        //

        //PID Correction
        Pose err = targetPose.subtract(robotPose);
        double distance = Math.hypot(err.getX(), err.getY());
        double calculatedCos = err.getX() / distance;
        double calculatedSin = err.getY() / distance;
        tpid.setPID(Constants.DriveCorrectionCoefficients.tPIDCoeff_GoToPoint.p, Constants.DriveCorrectionCoefficients.tPIDCoeff_GoToPoint.i, Constants.DriveCorrectionCoefficients.tPIDCoeff_GoToPoint.d);
        double translationalPower = tpid.calculate(-distance, 0);
        Vector pidVector = new Vector(calculatedCos, calculatedSin).scalarMultiply(translationalPower);
        correctionVector = correctionVector.add(pidVector).scalarMultiply(CorrectionMagnitude);
        //


        //Heading Correction
        double headingDiff = angleWrapper(err.getHeading());
        hpid.setPID(Constants.DriveCorrectionCoefficients.hPIDCoeff.p, Constants.DriveCorrectionCoefficients.hPIDCoeff.i, Constants.DriveCorrectionCoefficients.hPIDCoeff.d);
        double headingPower = hpid.calculate(-headingDiff, 0);
        //


        //Adding up every vector
        finalPower = pathingPower.add(correctionVector);
        finalPower.setHeading(headingPower * HeadingMagnitude);
        finalPower = finalPower.rotate(robotPose.getHeading());
        //


        //Scaling down
        finalPower = new Vector(finalPower.getX() * lateralMultiplier, finalPower.getY() * forwardMultiplier, finalPower.getHeading() * headingMultiplier);
        if (Math.abs(finalPower.getX()) + Math.abs(finalPower.getY()) + Math.abs(finalPower.getHeading()) > 1)
            finalPower = finalPower.scaleToMagnitude_AngularAsWell(1);
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
