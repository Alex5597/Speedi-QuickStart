package org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Localizer;

import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.DeviceNames.pinPointName;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.cmPerTickForward;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.cmPerTickLateral;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.parYEncoderLateralDistanceToCenterOfRotation;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.perpXEncoderForwardDistanceToCenterOfRotation;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.shouldUsePhysicalBraking;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.xDeceleration;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.yDeceleration;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.core.Util.Algorithm.LowPassFilter;
import org.firstinspires.ftc.teamcode.core.Util.Hardware.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.core.Util.Hardware.VelocityAdapter;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;
import org.firstinspires.ftc.teamcode.core.Util.Math.Vector;
import org.firstinspires.ftc.teamcode.core.Util.utils.Globals;

public class PinPointLocalizer implements Localizer {
    public GoBildaPinpointDriver odo;
    Telemetry telemetry;
    Pose currentPosition, predictedPose, lastPosition;
    double lastRawHeadingVel = 0, headingVelOffset = 0;
    double headingVel = 0;
    Vector velocityVectorRaw = new Vector(0, 0, 0);
    Vector glideVector = new Vector(0, 0, 0), lastVelocityVector = new Vector(0, 0, 0);
    LowPassFilter xVelocityFilter = new LowPassFilter(0.8, 0), yVelocityFilter = new LowPassFilter(0.8, 0);
    VelocityAdapter velocityAdapter;
    private boolean firstLoop = true;

    public PinPointLocalizer(HardwareMap hardwareMap, Pose startPose) {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, pinPointName);
        odo.setOffsets(perpXEncoderForwardDistanceToCenterOfRotation, parYEncoderLateralDistanceToCenterOfRotation); //TODO MM departare de la fiecare odopod la centru de rotatie

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);//COUNTS_PER_REVOLUTION / CIRCUMFERENCE OF THE WHEEL

        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);//TODO
        odo.resetPosAndIMU();
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        odo.setPosition(startPose.rotateFieldCoordinate(Math.PI / 2).toPose2D());
        currentPosition = startPose;
        lastPosition = startPose;
        predictedPose = startPose;
    }

    public PinPointLocalizer(HardwareMap hardwareMap, Pose startPose, Telemetry telemetry) {
        this.telemetry = telemetry;
        odo = hardwareMap.get(GoBildaPinpointDriver.class, pinPointName);

        odo.setOffsets(perpXEncoderForwardDistanceToCenterOfRotation, parYEncoderLateralDistanceToCenterOfRotation); //TODO MM departare de la fiecare odopod la centru de rotatie

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);//COUNTS_PER_REVOLUTION / CIRCUMFERENCE OF THE WHEEL IN MM

        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);//TODO;
        odo.resetPosAndIMU();
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        odo.setPosition(startPose.rotateFieldCoordinate(Math.PI / 2).toPose2D());

        currentPosition = startPose;
        lastPosition = startPose;
        predictedPose = startPose;
    }

    @Override
    public void update() {
        if (shouldUsePhysicalBraking)
            if (xDeceleration == 0 || yDeceleration == 0)
                throw new RuntimeException("Change xDeceleration and yDeceleration from 0");
        if (firstLoop) {
            velocityAdapter = new VelocityAdapter();

            xVelocityFilter.resetFilter(0);
            yVelocityFilter.resetFilter(0);

            firstLoop = false;
        }
        odo.update();

        double angle = angleWrapper(odo.getHeading());
        Pose2D pose = odo.getPosition();
        currentPosition = new Pose(pose).rotateFieldCoordinate(-Math.PI / 2);
        Vector velocity = velocityAdapter.getVelocity(currentPosition);
        velocityVectorRaw = new Vector(xVelocityFilter.getValue(velocity.getX()), yVelocityFilter.getValue(velocity.getY()), velocity.getHeading());
        glideVector = new Vector(
                Math.signum(velocityVectorRaw.getX()) * velocityVectorRaw.getX() * velocityVectorRaw.getX() / (2.0 * xDeceleration),
                Math.signum(velocityVectorRaw.getY()) * velocityVectorRaw.getY() * velocityVectorRaw.getY() / (2.0 * yDeceleration),
                0);
        predictedPose = currentPosition.add(glideVector.toPose());

        if (predictedPose.isNaN()) {
            Globals.emergencyStop = true;
            predictedPose = lastPosition;
        } else
            lastPosition = currentPosition;

        if (currentPosition.isNaN())
            currentPosition = lastPosition;
        if (glideVector.isNaN())
            glideVector = new Vector(
                    Math.signum(lastVelocityVector.getX()) * lastVelocityVector.getX() * lastVelocityVector.getX() / (2.0 * xDeceleration),
                    Math.signum(lastVelocityVector.getY()) * lastVelocityVector.getY() * lastVelocityVector.getY() / (2.0 * yDeceleration),
                    0);
        if(!velocityVectorRaw.isNaN())
            lastVelocityVector = velocityVectorRaw;
    }

    public Pose getLastPosition() {
        return lastPosition;
    }

    @Override
    public Vector getVelocity() {
        return velocityVectorRaw;
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
            Thread.sleep(400);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        odo.setPosition(startPose.toPose2D());

        velocityAdapter = new VelocityAdapter();
        xVelocityFilter.resetFilter(0);
        yVelocityFilter.resetFilter(0);
        lastPosition = startPose;
    }

    @Override
    public void updateOnlyImu() {
        odo.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);
        double angle = angleWrapper(odo.getHeading());
        currentPosition = new Pose(0, 0,DistanceUnit.CM, angle, AngleUnit.RADIANS);
    }

    @Override
    public Vector getGlideVector() {
        return glideVector;
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
