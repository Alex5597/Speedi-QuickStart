package org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Follower;

import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.DriveCorrectionCoefficients.hPIDCoeff;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.DriveCorrectionCoefficients.tPIDCoeff_Spline;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.FollowerConstants.resolution;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.FollowerConstants.shouldBrake;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.WAIT_TIME_VARIABLE;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.core.Util.Algorithm.SplineGenerator.Spline;
import org.firstinspires.ftc.teamcode.core.Util.Algorithm.SquidController;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;
import org.firstinspires.ftc.teamcode.core.Util.Math.Vector;
import org.firstinspires.ftc.teamcode.core.Util.utils.Constants;

@Config
public class SplineFollowing {
    public Spline trajectory;
    public static PIDController tPid = new PIDController(tPIDCoeff_Spline.p, tPIDCoeff_Spline.i, tPIDCoeff_Spline.d);
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
        trajectory.setFirstHeading(startPose.getHeading(AngleUnit.RADIANS));

        //xPid.setPID(xPIDCoeff_Spline.p, xPIDCoeff_Spline.i, xPIDCoeff_Spline.d);
        //yPid.setPID(yPIDCoeff_Spline.p, yPIDCoeff_Spline.i, yPIDCoeff_Spline.d);
        tPid.setPIDF(tPIDCoeff_Spline.p, tPIDCoeff_Spline.i, tPIDCoeff_Spline.d, 0);
        hPid.setPID(hPIDCoeff.p, hPIDCoeff.i, hPIDCoeff.d);

        // xPid.reset();
        // yPid.reset();
        tPid.reset();
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
        trajectory.setFirstHeading(startPose.getHeading(AngleUnit.RADIANS));

        //xPid.setPID(xPIDCoeff_Spline.p, xPIDCoeff_Spline.i, xPIDCoeff_Spline.d);
        //yPid.setPID(yPIDCoeff_Spline.p, yPIDCoeff_Spline.i, yPIDCoeff_Spline.d);
        tPid.setPIDF(tPIDCoeff_Spline.p, tPIDCoeff_Spline.i, tPIDCoeff_Spline.d, 0);
        hPid.setPID(hPIDCoeff.p, hPIDCoeff.i, hPIDCoeff.d);

        //xPid.reset();
        //yPid.reset();
        tPid.reset();
        hPid.reset();
    }

    public SplineFollowing(@NonNull Spline trajectory, Telemetry telemetry, boolean tangential) {
        this.trajectory = trajectory;
        this.telemetry = telemetry;

        this.instantHeading = false;
        this.tangential = tangential;

        finalPoint = trajectory.calculate(1);
        goToPoint = false;

        //xPid.setPID(xPIDCoeff_Spline.p, xPIDCoeff_Spline.i, xPIDCoeff_Spline.d);
        //yPid.setPID(yPIDCoeff_Spline.p, yPIDCoeff_Spline.i, yPIDCoeff_Spline.d);
        tPid.setPIDF(tPIDCoeff_Spline.p, tPIDCoeff_Spline.i, tPIDCoeff_Spline.d, 0);
        hPid.setPID(hPIDCoeff.p, hPIDCoeff.i, hPIDCoeff.d);

        //xPid.reset();
        //yPid.reset();
        tPid.reset();
        hPid.reset();
    }

    public Vector getMotorPower(Pose robotPose, Vector velocityVector, Vector glideVector) {
        if (goToPoint)
            return new Vector(WAIT_TIME_VARIABLE, WAIT_TIME_VARIABLE);

        tPid.setPIDF(tPIDCoeff_Spline.p, tPIDCoeff_Spline.i, tPIDCoeff_Spline.d, 0);
        hPid.setPID(hPIDCoeff.p, hPIDCoeff.i, hPIDCoeff.d);

        // Finding closest point
        double currentT = trajectory.findClosestPoint(robotPose.toVec(), lastT);
        lastT = currentT;
        Vector currTargetPoint = trajectory.calculate(currentT + 1.0 / resolution);
        Pose targetPose = new Pose(currTargetPoint, trajectory.heading(currentT + 1.0 / resolution));
        if (tangential)
            targetPose.setHeading(trajectory.heading(currentT + 1.0 / resolution),AngleUnit.RADIANS);
        else if (!instantHeading)
            targetPose.setHeading(hlerp(robotPose.getHeading(AngleUnit.RADIANS), trajectory.heading(currentT + 1.0 / resolution), tLerp), AngleUnit.RADIANS);

        // Check for final adjustment
        if (currentT >= 0.9 && trajectory.getLength() - trajectory.getLengthAt(currentT) <= glideVector.getMagnitude() && shouldBrake) {
            goToPoint = true;
            telemetry.addLine("GoToPoint activated");
            telemetry.update();
            return new Vector(WAIT_TIME_VARIABLE, WAIT_TIME_VARIABLE);
        }

        double curvature = trajectory.curvatureOfThePath(currentT);
        Vector centripetalCorrectionVector = Vector.polar(Range.clip(Constants.FollowerConstants.CentripetalScalingFactor * Constants.FollowerConstants.TotalMassOfRobot * Math.pow(Vector.dot(trajectory.firstDerivative(currentT).scaleToMagnitude(1), velocityVector), 2) * curvature, -0.8, 0.8), trajectory.firstDerivative(currentT).getRelativeHeading() + Math.PI / 2 * Math.signum(trajectory.pathNormalVect(currentT).getRelativeHeading()));
        telemetry.addData("Centripetal vect", centripetalCorrectionVector.toString());

        Pose err = targetPose.subtract(robotPose);
        double distance = Math.hypot(err.getX(DistanceUnit.CM), err.getY(DistanceUnit.CM));

        double calculatedCos = err.getX(DistanceUnit.CM) / distance;
        double calculatedSin = err.getY(DistanceUnit.CM) / distance;
        double translationalPower = tPid.calculate(-distance, 0);
        Vector pidCorrectionVector = new Vector(translationalPower * calculatedCos, translationalPower * calculatedSin).scaleToMagnitude(1);
        telemetry.addData("Pid vect", pidCorrectionVector.toString());

        Vector finalPower = centripetalCorrectionVector.add(pidCorrectionVector);
        if (finalPower.getMagnitude() >= 1) {
            double scale = Vector.findScaleFactor(centripetalCorrectionVector, pidCorrectionVector);
            pidCorrectionVector = pidCorrectionVector.scalarMultiply(scale);
            return centripetalCorrectionVector.add(pidCorrectionVector);
        }

        double headingDiff = angleWrapper(err.getHeading(AngleUnit.RADIANS));
        double headingPower = -hPid.calculate(-headingDiff, 0);
        finalPower.setHeading(headingPower);
        telemetry.addData("Heading power", headingPower);
        if (finalPower.getMagnitude_AngularAsWell() >= 1) {
            return finalPower.scaleToMagnitude_AngularAsWell(1);
        }

        Vector speedAlongPath = Vector.polar(Math.abs(1 - finalPower.getMagnitude_AngularAsWell()), trajectory.firstDerivative(currentT).getRelativeHeading());
        telemetry.addData("Speed vect", speedAlongPath.toString());
        finalPower = finalPower.add(speedAlongPath);

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
