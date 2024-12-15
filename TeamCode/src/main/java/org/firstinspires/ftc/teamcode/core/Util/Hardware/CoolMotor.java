package org.firstinspires.ftc.teamcode.core.Util.Hardware;

import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.SWITCH_FROM_STATIC_TO_KINETIC_FRICTION;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.core.Util.utils.Globals;

public class CoolMotor {
    double lastPower = 0;
    public double power = 0;
    public DcMotorEx motor;
    private double multipier;
    public String name;
    private final double epsilon = 1e-5;
    private double minPowerToOvercomeStaticFriction = 0.0;
    private double minPowerToOvercomeKineticFriction = 0.0;
    private long lastZeroTime = 0;

    public CoolMotor(DcMotorEx motor, String name) {
        this(motor, name, 1);
    }

    public CoolMotor(DcMotorEx motor, String name, double multiplier) {
        this.motor = motor;
        this.name = name;

        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        motor.setMotorType(motorConfigurationType);

        minPowerToOvercomeStaticFriction = 0.0;
        minPowerToOvercomeKineticFriction = 0.0;
        lastZeroTime = System.currentTimeMillis();
        this.multipier = multiplier;
    }

    public void setMinimumPowerToOvercomeStaticFriction(double value) {
        minPowerToOvercomeStaticFriction = value;
    }

    public void setMinimumPowerToOvercomeKineticFriction(double value) {
        minPowerToOvercomeKineticFriction = value;
    }

    public double getMinimumPowerToOvercomeStaticFriction() {
        return minPowerToOvercomeStaticFriction;
    }

    public double getMinimumPowerToOvercomeKineticFriction() {
        return minPowerToOvercomeKineticFriction;
    }

    public void setTargetPower(double power) {
        if (lastPower == 0) {
            lastZeroTime = System.currentTimeMillis();
        }
        power = Range.clip(power, -1.0, 1.0);
        double m = (System.currentTimeMillis() > SWITCH_FROM_STATIC_TO_KINETIC_FRICTION + lastZeroTime ? minPowerToOvercomeKineticFriction : minPowerToOvercomeStaticFriction) * (12.0 / Globals.voltage);
        power *= (1 - m);
        this.power = power + m * Math.signum(power);
    }

    double k = 0.7; // 0.5

    public void setTargetPowerSmooth(double power) {
        if (lastPower == 0) {
            lastZeroTime = System.currentTimeMillis();
        }
        if (power == 0) {
            this.power = 0;
            return;
        }
        power = Range.clip(power, -1.0, 1.0);
        double m = (System.currentTimeMillis() > SWITCH_FROM_STATIC_TO_KINETIC_FRICTION + lastZeroTime ? minPowerToOvercomeKineticFriction : minPowerToOvercomeStaticFriction) * (13.5 / Globals.voltage);
        power *= 1 - m;
        power = power + m * Math.signum(power);
        this.power = power * k + this.lastPower * (1 - k);
    }

    public void setPowerForced(double power) {
        this.power = power;
        if (power == 0)
            lastZeroTime = System.currentTimeMillis();
        if (Math.abs(power - lastPower) >= epsilon) {
            motor.setPower(power * multipier);
        }
        lastPower = power;
    }

    public double getPower() {
        return power;
    }

    public double getVelocity() {
        return motor.getVelocity();
    }

    public void update() {
        if (Math.abs(power - lastPower) >= epsilon) {
            motor.setPower(power * multipier);
        }
        if (power == 0) {
            lastZeroTime = System.currentTimeMillis();
        }
        lastPower = power;
    }
}