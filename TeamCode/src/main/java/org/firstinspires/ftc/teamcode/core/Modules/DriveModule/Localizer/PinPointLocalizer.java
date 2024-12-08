package org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Localizer;

import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.cmPerTickForward;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.cmPerTickLateral;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.parYEncoderLateralDistanceToCenterOfRotation;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.perpXEncoderForwardDistanceToCenterOfRotation;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.xDeceleration;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.yDeceleration;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.core.Util.Algorithm.LowPassFilter;
import org.firstinspires.ftc.teamcode.core.Util.Hardware.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.core.Util.Hardware.VelocityAdapterForPinPointAndRev;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;
import org.firstinspires.ftc.teamcode.core.Util.Math.Vector;

public class PinPointLocalizer implements Localizer {
    public GoBildaPinpointDriver odo;
    Telemetry telemetry;
    Pose currentPosition, predictedPose;
    double lastRawHeadingVel = 0, headingVelOffset = 0;
    double headingVel = 0;
    Vector velocityVectorRaw = new Vector(0, 0, 0);
    Vector velocityVector = new Vector(0, 0, 0), lastVelocityVector = new Vector(0, 0, 0);
    LowPassFilter xVelocityFilter = new LowPassFilter(0.8, 0), yVelocityFilter = new LowPassFilter(0.8, 0);
    VelocityAdapterForPinPointAndRev adapterX = new VelocityAdapterForPinPointAndRev(), adapterY = new VelocityAdapterForPinPointAndRev();

    public PinPointLocalizer(HardwareMap hardwareMap, Pose startPose) {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(perpXEncoderForwardDistanceToCenterOfRotation, parYEncoderLateralDistanceToCenterOfRotation); //TODO MM departare de la fiecare odopod la centru de rotatie

        odo.setEncoderResolution(8192.00 / (35.0 * Math.PI));//COUNTS_PER_REVOLUTION / CIRCUMFERENCE OF THE WHEEL

        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        odo.setPosition(startPose.toPose2D());
        currentPosition = startPose;
        predictedPose = startPose;
    }

    public PinPointLocalizer(HardwareMap hardwareMap, Pose startPose, Telemetry telemetry) {
        this.telemetry = telemetry;
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        odo.setOffsets(perpXEncoderForwardDistanceToCenterOfRotation, parYEncoderLateralDistanceToCenterOfRotation); //TODO MM departare de la fiecare odopod la centru de rotatie

        odo.setEncoderResolution(8192.00 / (35.0 * Math.PI));//COUNTS_PER_REVOLUTION / CIRCUMFERENCE OF THE WHEEL

        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        odo.setPosition(startPose.toPose2D());

        currentPosition = startPose;
        predictedPose = startPose;
    }

    @Override
    public void update() {
        odo.update();

        double angle = angleWrapper(odo.getHeading());
        Pose2D pose = odo.getPosition();
        currentPosition = new Pose(pose.getX(DistanceUnit.CM), pose.getY(DistanceUnit.CM), angle);

        double xVelocity = odo.getVelX();//(adapterX.getVelocityBasedOnTicks(-odo.getEncoderX())) * cmPerTickForward;
        double yVelocity = odo.getVelY();//(adapterY.getVelocityBasedOnTicks(odo.getEncoderY())) * cmPerTickForward;

        velocityVectorRaw = new Vector(xVelocityFilter.getValue(xVelocity), yVelocityFilter.getValue(yVelocity));
        Vector driveTrainvelocity = velocityVectorRaw.rotate(-currentPosition.getHeading()); //E acelasi lucru cu driveTrainVelocity = velocity, dar asa e corect dpdv geometric
        lastVelocityVector = velocityVector;
        velocityVector = new Vector(
                Math.signum(driveTrainvelocity.getX()) * driveTrainvelocity.getX() * driveTrainvelocity.getX() / (2.00 * xDeceleration),
                Math.signum(driveTrainvelocity.getY()) * driveTrainvelocity.getY() * driveTrainvelocity.getY() / (2.00 * yDeceleration),
                0);
        velocityVector = velocityVector.rotate(-currentPosition.getHeading());
        predictedPose = currentPosition.add(velocityVector.toPose());

        if (predictedPose.isNaN())
            predictedPose = currentPosition;
        if (velocityVector.isNaN())
            velocityVector = lastVelocityVector;
    }

    @Override
    public Vector getVelocity() {
        return new Vector(odo.getVelocity());
    }

    @Override
    public Pose getPoseEstimate() {
        return currentPosition;
    }

    @Override
    public Pose getPredictedPoseEstimate() {
        return predictedPose;
    }

    @Override
    public void resetPosition(Pose startPose) {
        odo.resetPosAndIMU();
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        odo.setPosition(startPose.toPose2D());
    }

    @Override
    public void updateOnlyImu() {
        odo.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);
    }

    @Override
    public Vector getVelocityVector() {
        return velocityVector;
    }

    @Override
    public Vector getRawVelocity() {
        return new Vector(odo.getVelX() * 10.0 / cmPerTickForward, odo.getVelY() * 10.0 / cmPerTickLateral); //Ticks/s
    }

    private double angleWrapper(double angle) {
        angle %= (2.0 * Math.PI);
        if (angle > Math.PI) angle -= 2.0 * Math.PI;
        if (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }
}
