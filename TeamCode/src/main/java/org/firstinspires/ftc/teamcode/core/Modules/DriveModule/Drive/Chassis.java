package org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Drive;

import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.DeviceNames.leftBackMotorName;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.DeviceNames.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.DeviceNames.rightBackMotorName;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.DeviceNames.rightFrontMotorName;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.MecanumChassisConstants.forwardChassisMaxVelocity;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.MecanumChassisConstants.lateralChassisMaxVelocity;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.MecanumChassisConstants.minPowerToOvercomeKineticFriction;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.MecanumChassisConstants.minPowersToOvercomeStaticFriction;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.MecanumChassisConstants.shouldReverseLeftBackMotor;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.MecanumChassisConstants.shouldReverseLeftForwardMotor;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.MecanumChassisConstants.shouldReverseRightBackMotor;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.MecanumChassisConstants.shouldReverseRightForwardMotor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.Modules.Module;
import org.firstinspires.ftc.teamcode.core.Util.Hardware.CoolMotor;
import org.firstinspires.ftc.teamcode.core.Util.Math.Vector;
import org.firstinspires.ftc.teamcode.core.Util.utils.Globals;

import java.util.Arrays;
import java.util.List;

public class Chassis implements Module {
    public CoolMotor LFM, RFM, RBM, LBM;
    public List<CoolMotor> motors;
    ElapsedTime voltageTimer = new ElapsedTime();
    HardwareMap hardwareMap;
    boolean brake;
    double[] wheelSpeeds = new double[4];
    private double maxPower = 1;
    double driveMultiplier = 1;
    final Vector copiedFrontLeftVector = new Vector(-lateralChassisMaxVelocity, forwardChassisMaxVelocity).scaleToMagnitude(1);
    final Vector[] mecanumVectors = new Vector[]{
            Vector.polar(copiedFrontLeftVector.getMagnitude(), copiedFrontLeftVector.getRelativeHeading()),
            Vector.polar(copiedFrontLeftVector.getMagnitude(), 2 * Math.PI - copiedFrontLeftVector.getRelativeHeading()),
            Vector.polar(copiedFrontLeftVector.getMagnitude(), 2 * Math.PI - copiedFrontLeftVector.getRelativeHeading()),
            Vector.polar(copiedFrontLeftVector.getMagnitude(), copiedFrontLeftVector.getRelativeHeading())
    };

    public Chassis(HardwareMap hardwareMap, boolean brake) {
        this.brake = brake;
        this.hardwareMap = hardwareMap;
        LFM = new CoolMotor(hardwareMap.get(DcMotorEx.class, leftFrontMotorName), "left front motor");
        LBM = new CoolMotor(hardwareMap.get(DcMotorEx.class, leftBackMotorName), "left back motor");
        RFM = new CoolMotor(hardwareMap.get(DcMotorEx.class, rightFrontMotorName), "right front motor");
        RBM = new CoolMotor(hardwareMap.get(DcMotorEx.class, rightBackMotorName), "right back motor");
        if (brake) {
            LFM.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LBM.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RBM.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RFM.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (shouldReverseLeftBackMotor)
            LBM.motor.setDirection(DcMotorEx.Direction.REVERSE);
        if (shouldReverseRightForwardMotor)
            RFM.motor.setDirection(DcMotorEx.Direction.REVERSE);
        if (shouldReverseLeftForwardMotor)
            LFM.motor.setDirection(DcMotorEx.Direction.REVERSE);
        if (shouldReverseRightBackMotor)
            RBM.motor.setDirection(DcMotorEx.Direction.REVERSE);
        motors = Arrays.asList(LFM, LBM, RFM, RBM);

        Globals.voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        voltageTimer.reset();
        setMinPowersToOvercomeFriction();
    }

    public void setMotorPowerForced(Vector input) {
        wheelSpeeds[0] = input.getY() + input.getX() + input.getHeading();//LEFT FRONT
        wheelSpeeds[1] = input.getY() - input.getX() + input.getHeading();//LEFT BACK
        wheelSpeeds[2] = input.getY() - input.getX() - input.getHeading();//RIGHT FRONT
        wheelSpeeds[3] = input.getY() + input.getX() - input.getHeading();//RIGHT BACK

        double max = 1;
        for (double wheelSpeed : wheelSpeeds)
            max = Math.max(max, Math.abs(wheelSpeed));

        if (max > maxPower) {
            wheelSpeeds[0] = (wheelSpeeds[0] / max) * maxPower;
            wheelSpeeds[1] = (wheelSpeeds[1] / max) * maxPower;
            wheelSpeeds[2] = (wheelSpeeds[2] / max) * maxPower;
            wheelSpeeds[3] = (wheelSpeeds[3] / max) * maxPower;
        }

        for (int i = 0; i < 4; i++)
            motors.get(i).setPowerForced(wheelSpeeds[i]);
    }

    public void setMotorPower(Vector input) {
        wheelSpeeds[0] = input.getY() + input.getX() + input.getHeading();//LEFT FRONT
        wheelSpeeds[1] = input.getY() - input.getX() + input.getHeading();//LEFT BACK
        wheelSpeeds[2] = input.getY() - input.getX() - input.getHeading();//RIGHT FRONT
        wheelSpeeds[3] = input.getY() + input.getX() - input.getHeading();//RIGHT BACK

        double max = 1;
        for (double wheelSpeed : wheelSpeeds)
            max = Math.max(max, Math.abs(wheelSpeed));

        if (max > maxPower) {
            wheelSpeeds[0] = (wheelSpeeds[0] / max) * maxPower;
            wheelSpeeds[1] = (wheelSpeeds[1] / max) * maxPower;
            wheelSpeeds[2] = (wheelSpeeds[2] / max) * maxPower;
            wheelSpeeds[3] = (wheelSpeeds[3] / max) * maxPower;
        }

        for (int i = 0; i < 4; i++)
            motors.get(i).setTargetPowerSmooth(wheelSpeeds[i]);
    }

    public void setMotorPower(Vector[] truePathingVectors) {
        truePathingVectors[0] = truePathingVectors[0].scalarMultiply(2.0);
        truePathingVectors[1] = truePathingVectors[1].scalarMultiply(2.0);

        wheelSpeeds[0] = (-mecanumVectors[1].getY() * truePathingVectors[0].getY() - truePathingVectors[0].getX() * mecanumVectors[1].getX()) / (mecanumVectors[1].getY() * mecanumVectors[0].getX() - mecanumVectors[0].getY() * mecanumVectors[1].getX());
        wheelSpeeds[1] = (-mecanumVectors[0].getY() * truePathingVectors[0].getY() - truePathingVectors[0].getX() * mecanumVectors[0].getX()) / (mecanumVectors[0].getY() * mecanumVectors[1].getX() - mecanumVectors[1].getY() * mecanumVectors[0].getX());
        wheelSpeeds[2] = (-mecanumVectors[3].getY() * truePathingVectors[1].getY() - truePathingVectors[1].getX() * mecanumVectors[3].getX()) / (mecanumVectors[3].getY() * mecanumVectors[2].getX() - mecanumVectors[2].getY() * mecanumVectors[3].getX());
        wheelSpeeds[3] = (-mecanumVectors[2].getY() * truePathingVectors[1].getY() - truePathingVectors[1].getX() * mecanumVectors[2].getX()) / (mecanumVectors[2].getY() * mecanumVectors[3].getX() - mecanumVectors[3].getY() * mecanumVectors[2].getX());

        double max = 1;
        for (double wheelSpeed : wheelSpeeds)
            max = Math.max(max, Math.abs(wheelSpeed));

        if (max > maxPower) {
            wheelSpeeds[0] = (wheelSpeeds[0] / max) * maxPower;
            wheelSpeeds[1] = (wheelSpeeds[1] / max) * maxPower;
            wheelSpeeds[2] = (wheelSpeeds[2] / max) * maxPower;
            wheelSpeeds[3] = (wheelSpeeds[3] / max) * maxPower;
        }

        for (int i = 0; i < 4; i++)
            motors.get(i).setTargetPower(wheelSpeeds[i]);//TODO ai schimbat cv aici
    }

    public void setMotorPower(double frontLeftPower, double backLeftPower, double frontRightPower, double backRightPower) {
        motors.get(0).setPowerForced(frontLeftPower);
        motors.get(1).setPowerForced(backLeftPower);
        motors.get(2).setPowerForced(frontRightPower);
        motors.get(3).setPowerForced(backRightPower);
    }

    public void drive(Gamepad gamepad) {
        resetMinPowersToOvercomeFriction();

        double forward = -gamepad.left_stick_y * driveMultiplier;
        double strafe = gamepad.left_stick_x * driveMultiplier;
        double turn = gamepad.right_stick_x * driveMultiplier;

        Vector drive = new Vector(strafe, forward, turn);
        if (drive.getMagnitude() <= 0.05) {
            drive.scalarMultiply(0);
        }

        //setMotorPower(new Vector[]{new Vector(drive.getX(), Range.clip(drive.getY() + drive.getHeading(), -1, 1)), new Vector(drive.getX(), Range.clip(drive.getY() - drive.getHeading(), -1, 1))});
        setMotorPowerForced(drive);
    }

    public void driveFieldCentric(Gamepad gamepad, double angle) {
        resetMinPowersToOvercomeFriction();

        double forward = (-gamepad.left_stick_y * driveMultiplier);
        double strafe = (gamepad.left_stick_x * driveMultiplier);
        double turn = (gamepad.right_stick_x * driveMultiplier);

        Vector drive = new Vector(strafe, forward, turn).rotate(angle);
        if (drive.getMagnitude() <= 0.05) {
            drive.scalarMultiply(0);
        }

        //setMotorPower(new Vector[]{new Vector(drive.getX(), Range.clip(drive.getY() + drive.getHeading(), -1, 1)), new Vector(drive.getX(), Range.clip(drive.getY() - drive.getHeading(), -1, 1))});
        setMotorPowerForced(drive);
    }

    public double smoothControls(double value) {
        return 0.5 * Math.tan(1.12 * value);
    }

    public void resetMinPowersToOvercomeFriction() {
        if (motors.get(0).getMinimumPowerToOvercomeKineticFriction() == 0.0)
            return;

        for (CoolMotor m : motors) {
            m.setMinimumPowerToOvercomeStaticFriction(0.0);
            m.setMinimumPowerToOvercomeKineticFriction(0.0);
        }
    }

    public void setMinPowersToOvercomeFriction() {
        if (motors.get(0).getMinimumPowerToOvercomeKineticFriction() == minPowerToOvercomeKineticFriction)
            return;

        for (int i = 0; i < 4; i++) {
            motors.get(i).setMinimumPowerToOvercomeStaticFriction(minPowersToOvercomeStaticFriction[i]);
            motors.get(i).setMinimumPowerToOvercomeKineticFriction(minPowerToOvercomeKineticFriction);
        }
    }

    public void goAtMinimumPower() {
        for (int i = 0; i < 4; i++)
            motors.get(i).setPowerForced(minPowersToOvercomeStaticFriction[i]);
    }

    @Override
    public void update() {
        if (voltageTimer.milliseconds() >= 300) {
            Globals.voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            voltageTimer.reset();
        }

        for (CoolMotor m : motors) {
            m.update();
        }
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

    public void setDriveMultiplier(double driveMultiplier) {
        this.driveMultiplier = driveMultiplier;
    }

    public double getDriveMultiplier() {
        return driveMultiplier;
    }
}
