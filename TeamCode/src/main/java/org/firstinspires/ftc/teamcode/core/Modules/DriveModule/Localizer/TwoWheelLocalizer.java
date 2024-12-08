package org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Localizer;

import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.cmPerTickForward;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.cmPerTickLateral;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.parYTicks;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.perpXTicks;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.xDeceleration;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.yDeceleration;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Globals.isAuto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.teamcode.core.Modules.Module;
import org.firstinspires.ftc.teamcode.core.Util.Algorithm.LowPassFilter;
import org.firstinspires.ftc.teamcode.core.Util.Hardware.Encoder;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;
import org.firstinspires.ftc.teamcode.core.Util.Math.Vector;

/**
 * @noinspection NonAsciiCharacters
 */
public class TwoWheelLocalizer implements Localizer {
    Encoder par, perp;
    HardwareMap hardwareMap;
    IMU imu;
    Pose currentPosition;
    Pose predictedPose;
    public Vector
            velocityVector = new Vector(0, 0),
            velocity = new Vector(0, 0);
    LowPassFilter
            xVelocityFilter = new LowPassFilter(0.8, 0),
            yVelocityFilter = new LowPassFilter(0.8, 0);

    LowPassFilter
            xPositionFilter = new LowPassFilter(1, 0),
            yPositionFilter = new LowPassFilter(1, 0);
    double initialHeading;
    Telemetry telemetry;
    double prev_par_pos = 0, prev_perp_pos = 0, lastAngle;
    double startAngle;
    double lastRawHeadingVel = 0, headingVelOffset = 0;
    double headingVel = 0;

    public TwoWheelLocalizer(HardwareMap hardwareMap, Pose startPose, Telemetry telemetry) {
        currentPosition = startPose;
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        startAngle = startPose.getHeading();
        lastAngle = startAngle;

        par = new Encoder(hardwareMap.get(DcMotorEx.class, "LFM"));
        perp = new Encoder(hardwareMap.get(DcMotorEx.class, "RFM"));

        par.setDirection(Encoder.Direction.REVERSE);
        perp.setDirection(Encoder.Direction.FORWARD);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.resetDeviceConfigurationForOpMode();
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        imu.resetYaw();
        initialHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    @Override
    public void update() {
        double par_current_pos = xPositionFilter.getValue(par.getCurrentPosition());
        double perp_current_pos = yPositionFilter.getValue(perp.getCurrentPosition());
        double angle = startAngle + imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - initialHeading;

        double xVelocity = par.getCorrectedVelocity();
        double yVelocity = perp.getCorrectedVelocity();

        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS);
        double rawHeadingVel = angularVelocity.zRotationRate;
        if (Math.abs(rawHeadingVel - lastRawHeadingVel) > Math.PI) {
            headingVelOffset -= Math.signum(rawHeadingVel) * 2 * Math.PI;
        }
        lastRawHeadingVel = rawHeadingVel;
        headingVel = headingVelOffset + rawHeadingVel;

        double Δ_par_pos = par_current_pos - prev_par_pos;
        double Δ_perp_pos = perp_current_pos - prev_perp_pos;
        double Δ_theta = angle - lastAngle;
        Δ_theta = angleWrapper(Δ_theta);

        double dx = (Δ_par_pos - Δ_theta * parYTicks) * cmPerTickLateral;
        double dy = (Δ_perp_pos - Δ_theta * perpXTicks) * cmPerTickForward;

        double dxVelocity = (xVelocity - Δ_theta * headingVel) * cmPerTickLateral;
        double dyVelocity = (yVelocity - Δ_theta * headingVel) * cmPerTickForward;

        double a, b, c;
        if (Math.abs(Δ_theta) <= 10e-3) {
            a = 1 - (Δ_theta * Δ_theta) / 6;
            b = -Δ_theta / 2;
            c = Δ_theta / 2;
        } else {
            a = Math.sin(Δ_theta) / Δ_theta;
            b = (Math.cos(Δ_theta) - 1) / Δ_theta;
            c = (1 - Math.cos(Δ_theta)) / Δ_theta;
        }
        double d = Math.cos(angle), e = Math.sin(angle);

        currentPosition = currentPosition.add(
                new Pose(
                        -(dy * (a * d + e * b) + dx * (e * a + c * d)),
                        dy * (b * d - e * a) + dx * (a * d - e * c),
                        Δ_theta
                )
        );
        currentPosition.setHeading(angleWrapper(currentPosition.getHeading()));


        velocity = new Vector(yVelocityFilter.getValue(-dyVelocity), xVelocityFilter.getValue(dxVelocity));
        Vector driveTrainvelocity = velocity.rotate(0); //E acelasi lucru cu driveTrainVelocity = velocity, dar asa e corect dpdv geometric
        velocityVector = new Vector(
                Math.signum(driveTrainvelocity.getX()) * driveTrainvelocity.getX() * driveTrainvelocity.getX() / (5.08 * xDeceleration),
                Math.signum(driveTrainvelocity.getY()) * driveTrainvelocity.getY() * driveTrainvelocity.getY() / (5.08 * yDeceleration),
                0);
        velocityVector = velocityVector.rotate(-currentPosition.getHeading());
        predictedPose = currentPosition.add(velocityVector.toPose());

        prev_par_pos = par_current_pos;
        prev_perp_pos = perp_current_pos;
        lastAngle = angle;
    }

    @Override
    public void updateOnlyImu() {
        double angle = startAngle + imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - initialHeading;
        currentPosition.setHeading(angleWrapper(angle));
    }

    @Override
    public Vector getRawVelocity() {
        return new Vector(par.getCorrectedVelocity(), perp.getCorrectedVelocity());// ticks/s
    }

    @Override
    public Vector getVelocity() {
        return velocity;
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
        currentPosition = startPose;
        predictedPose = startPose;
        velocityVector = new Vector(0, 0, 0);

        imu.resetYaw();
        initialHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        startAngle = startPose.getHeading();

        if (isAuto) {
            par = new Encoder(hardwareMap.get(DcMotorEx.class, "LFM"), (int) par.getCurrentPosition());
            perp = new Encoder(hardwareMap.get(DcMotorEx.class, "RFM"), (int) perp.getCurrentPosition());

            par.setDirection(Encoder.Direction.REVERSE);
            perp.setDirection(Encoder.Direction.FORWARD);

            lastAngle = startAngle;
            prev_par_pos = par.getCurrentPosition();
            prev_perp_pos = perp.getCurrentPosition();

            xPositionFilter.resetFilter(prev_par_pos);
            yPositionFilter.resetFilter(prev_perp_pos);

            xVelocityFilter.resetFilter(0);
            yVelocityFilter.resetFilter(0);
        }
    }

    public double getInitialHeading() {
        return startAngle;
    }


    private double angleWrapper(double angle) {
        if (angle > Math.PI) angle -= 2.0 * Math.PI;
        if (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    @Override
    public Vector getVelocityVector() {
        return velocityVector;
    }
}