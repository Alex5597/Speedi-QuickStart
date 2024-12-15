package org.firstinspires.ftc.teamcode.core.Util.Hardware;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.Util.Algorithm.AsymmetricMotionProfiler;

public class BetterServo {
    Servo servo;
    AsymmetricMotionProfiler profile;
    public double cachedPosition, targetPosition;
    public boolean isSlower = false;
    private double lastPosition = -1;
    private final double epsilon = 1e-5;

    public BetterServo(Servo servo, double initialPosition) {
        this.servo = servo;
        this.isSlower = false;
        setInitialPosition(initialPosition);
    }

    public BetterServo(Servo servo, double profileMaxVelocity, double profileAcceleration, double profileDeceleration, double initialPosition) {
        this.servo = servo;
        this.isSlower = true;
        profile = new AsymmetricMotionProfiler(profileMaxVelocity, profileAcceleration, profileDeceleration);
        setInitialPosition(initialPosition);
    }

    public BetterServo(Servo servo) {
        this.servo = servo;
        this.isSlower = false;
    }

    private void setInitialPosition(double pos) {
        if (!isSlower) {
            if (Math.abs(pos - lastPosition) >= epsilon)
                servo.setPosition(pos);
            lastPosition = pos;
            return;
        }
        cachedPosition = pos;
        targetPosition = pos;
        profile.setMotion(pos, pos, 0);
        servo.setPosition(pos);
    }

    public void setPosition(double position) {
        if (!isSlower) {
            if (Math.abs(position - lastPosition) >= epsilon)
                servo.setPosition(position);
            lastPosition = position;
            return;
        }
        if (position == targetPosition) return;
        targetPosition = position;
        profile.setMotion(cachedPosition, targetPosition, profile.getSignedVelocity());
    }

    public void update() {
        if (!isSlower)
            return;
        profile.update();
        if (cachedPosition != profile.getPosition()) {
            cachedPosition = profile.getPosition();
            servo.setPosition(cachedPosition);
        }
    }

    public void updateIsSlower(boolean isSlower) {
        this.isSlower = isSlower;
    }

    public double getPosition() {
        return cachedPosition;
    }

    public void setPos(double pos) {
        cachedPosition = pos;
    }

    public double getVelo() {
        return profile.getVelocity();
    }
}
