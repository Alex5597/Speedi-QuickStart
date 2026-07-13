package org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Localizer;

import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.DeviceNames.pinPointName;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.GoToPointConstants.forwardDeceleration;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.GoToPointConstants.lateralDeceleration;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.GoToPointConstants.shouldUsePhysicalBraking;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LocalizerConstants.cmPerTickForward;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LocalizerConstants.cmPerTickLateral;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LocalizerConstants.xPodOffsetInMM;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LocalizerConstants.yPodOffsetInMM;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LocalizerConstants.shouldReverseForwardEncoder;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LocalizerConstants.shouldReverseLateralEncoder;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LocalizerConstants.typeOfEncoders;

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
    Vector velocityVectorRaw = new Vector(0, 0, 0);
    Vector glideVector = new Vector(0, 0, 0), lastVelocityVector = new Vector(0, 0, 0);
    LowPassFilter xVelocityFilter = new LowPassFilter(0.8, 0), yVelocityFilter = new LowPassFilter(0.8, 0);
    VelocityAdapter velocityAdapter;
    private boolean firstLoop = true;

    public PinPointLocalizer(HardwareMap hardwareMap, Pose startPose) {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, pinPointName);

        odo.setOffsets(xPodOffsetInMM, yPodOffsetInMM);//X pod offset = sideways(left+), Y pod offset = forward(forward+), see the GoBildaPinpointDriver.setOffsets docs
        odo.setEncoderResolution(typeOfEncoders);
        odo.setEncoderDirections(shouldReverseForwardEncoder ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD, shouldReverseLateralEncoder ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD);//TODO
        odo.resetPosAndIMU();
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        odo.setPosition(toPinpointFrame(startPose));
        currentPosition = startPose;
        lastPosition = startPose;
        predictedPose = startPose;
    }

    public PinPointLocalizer(HardwareMap hardwareMap, Pose startPose, Telemetry telemetry) {
        this.telemetry = telemetry;
        odo = hardwareMap.get(GoBildaPinpointDriver.class, pinPointName);

        odo.setOffsets(xPodOffsetInMM, yPodOffsetInMM);//X pod offset = sideways(left+), Y pod offset = forward(forward+), see the GoBildaPinpointDriver.setOffsets docs
        odo.setEncoderResolution(typeOfEncoders);
        odo.setEncoderDirections(shouldReverseForwardEncoder ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD, shouldReverseLateralEncoder ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD);//TODO
        odo.resetPosAndIMU();
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        odo.setPosition(toPinpointFrame(startPose));

        currentPosition = startPose;
        lastPosition = startPose;
        predictedPose = startPose;
    }

    @Override
    public void update() {
        if (shouldUsePhysicalBraking)
            if (lateralDeceleration == 0 || forwardDeceleration == 0)
                throw new RuntimeException("Change xDeceleration and yDeceleration from 0");
        odo.update();

        Pose2D pose = odo.getPosition();
        currentPosition = fromPinpointFrame(pose);
        if (firstLoop) {
            velocityAdapter = new VelocityAdapter(currentPosition);
            xVelocityFilter.resetFilter(0);
            yVelocityFilter.resetFilter(0);
            firstLoop = false;
        }
        Vector velocity = velocityAdapter.getVelocity(currentPosition);
        velocityVectorRaw = new Vector(xVelocityFilter.getValue(velocity.getX()), yVelocityFilter.getValue(velocity.getY()), velocity.getHeading());
        glideVector = new Vector(
                Math.signum(velocityVectorRaw.getX()) * velocityVectorRaw.getX() * velocityVectorRaw.getX() / (2.0 * lateralDeceleration),
                Math.signum(velocityVectorRaw.getY()) * velocityVectorRaw.getY() * velocityVectorRaw.getY() / (2.0 * forwardDeceleration),
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
                    Math.signum(lastVelocityVector.getX()) * lastVelocityVector.getX() * lastVelocityVector.getX() / (2.0 * lateralDeceleration),
                    Math.signum(lastVelocityVector.getY()) * lastVelocityVector.getY() * lastVelocityVector.getY() / (2.0 * forwardDeceleration),
                    0);
        if (!velocityVectorRaw.isNaN())
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

        odo.setPosition(toPinpointFrame(startPose));

        currentPosition = startPose;
        predictedPose = startPose;
        velocityAdapter = new VelocityAdapter(startPose);
        xVelocityFilter.resetFilter(0);
        yVelocityFilter.resetFilter(0);
        lastPosition = startPose;
    }

    @Override
    public void updateOnlyImu() {
        odo.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);
        //the Pinpoint heading is CCW positive, SPEEDI headings are CW positive
        double angle = angleWrapper(-odo.getHeading());
        currentPosition = new Pose(0, 0, DistanceUnit.CM, angle, AngleUnit.RADIANS);
    }

    /**
     * Converts from the Pinpoint frame (x forward, y left, heading CCW positive) to the SPEEDI
     * frame (x right, y forward, heading CW positive)
     */
    private static Pose fromPinpointFrame(Pose2D pinpointPose) {
        Pose rotated = new Pose(pinpointPose).rotateFieldCoordinate(-Math.PI / 2);
        rotated.setHeading(-rotated.getHeading(AngleUnit.RADIANS), AngleUnit.RADIANS);
        return rotated;
    }

    private static Pose2D toPinpointFrame(Pose speediPose) {
        Pose rotated = speediPose.rotateFieldCoordinate(Math.PI / 2);
        rotated.setHeading(-speediPose.getHeading(AngleUnit.RADIANS), AngleUnit.RADIANS);
        return rotated.toPose2D();
    }

    @Override
    public Vector getGlideVector() {
        return glideVector;
    }

    @Override
    public Vector getRawVelocity() {
        return new Vector(odo.getVelX() * 10.0 / cmPerTickForward, odo.getVelY() * 10.0 / cmPerTickLateral); //Ticks/s
    }

    @Override
    public double getParallelEncPosRaw() {
        return odo.getEncoderX();
    }

    @Override
    public double getPerpendicularEncPosRaw() {
        return odo.getEncoderY();
    }

    private double angleWrapper(double angle) {
        angle %= (2.0 * Math.PI);
        if (angle > Math.PI) angle -= 2.0 * Math.PI;
        if (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }
}
