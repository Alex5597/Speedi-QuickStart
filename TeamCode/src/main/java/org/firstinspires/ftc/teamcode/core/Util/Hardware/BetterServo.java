package org.firstinspires.ftc.teamcode.core.Util.Hardware;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;

import org.firstinspires.ftc.teamcode.core.Util.Algorithm.AsymmetricMotionProfiler;

public class BetterServo {
    public Servo servo;
    ServoImpl servo1;
    AsymmetricMotionProfiler profile;
    public double cachedPosition, targetPosition;
    public boolean useMotionProfiler = false;
    private double lastPosition = -1;
    private final double epsilon = 1e-5;
    private double position;

    public BetterServo(Servo servo, double initialPosition) {
        this.servo = servo;
        this.useMotionProfiler = false;
        setInitialPosition(initialPosition);
    }

    public BetterServo(Servo servo, double initialPosition, boolean isAutonomous) {
        this.servo = servo;
        this.useMotionProfiler = false;
        setInitialPosition(initialPosition, isAutonomous);
    }

    public BetterServo(Servo servo, double profileMaxVelocity, double profileAcceleration, double profileDeceleration, double initialPosition) {
        this.servo = servo;
        this.useMotionProfiler = true;
        profile = new AsymmetricMotionProfiler(profileMaxVelocity, profileAcceleration, profileDeceleration);
        setInitialPosition(initialPosition);
    }

    public BetterServo(Servo servo, double profileMaxVelocity, double profileAcceleration, double profileDeceleration, double initialPosition, boolean isAutonomous) {
        this.servo = servo;
        this.useMotionProfiler = true;
        profile = new AsymmetricMotionProfiler(profileMaxVelocity, profileAcceleration, profileDeceleration);
        setInitialPosition(isAutonomous ? initialPosition : -1, isAutonomous);
    }

    public void setCoeff(double maxVelocity, double acceleration, double deceleration) {
        profile.setCoefficients(maxVelocity, acceleration, deceleration);
    }

    public BetterServo(Servo servo) {
        this.servo = servo;
        this.useMotionProfiler = false;
    }

    private void setInitialPosition(double pos) {
        if (!useMotionProfiler) {
            position = pos;
            return;
        }
        cachedPosition = pos;
        targetPosition = pos;
        profile.setMotion(pos, pos);
        servo.setPosition(pos);
    }

    private void setInitialPosition(double pos, boolean isAutonomous) {
        if (!useMotionProfiler) {
            if (isAutonomous) {
                this.position = pos;
                servo.setPosition(pos);
            }
            return;
        }
        cachedPosition = pos;
        targetPosition = pos;
        profile.setMotion(pos, pos);
        if (isAutonomous) {
            servo.setPosition(pos);
        }
    }

    public void setPosition(double position) {
        if (!useMotionProfiler) {
            this.position = position;
            return;
        }
        if (position == targetPosition){return;}
        targetPosition = position;
        profile.setMotion(cachedPosition, targetPosition);
    }

    public void update() {
        if (!useMotionProfiler) {
            if (Math.abs(position - lastPosition) >= epsilon) {
                servo.setPosition(position);
                lastPosition = position;
            }
            return;
        }
        if (cachedPosition != profile.getPosition()) {
            cachedPosition = profile.getPosition();
            servo.setPosition(cachedPosition);
        }
    }

    public void updateUseMotionProfiler(boolean useMotionProfiler) {
        this.useMotionProfiler = useMotionProfiler;
    }

    public double getTarget() {
        if (!useMotionProfiler)
            return position;
        else
            return cachedPosition;
    }

    public void setPos(double pos) {
        cachedPosition = pos;
    }

   // public double getVelo() {        return profile.getVelocity();}

}
