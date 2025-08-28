package org.firstinspires.ftc.teamcode.CodeBTC.Core.Algorithms;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AsymmetricMotionProfiler {
    private double T0, T1, T2;
    private double acceleration, deceleration, maxVelocity;
    private double maxUsedVelocity;
    public double initialPosition, finalPosition;
    ElapsedTime timer = new ElapsedTime();

    //private double position, velocity, signedVelocity;

    public AsymmetricMotionProfiler(double maxVelocity, double acceleration, double deceleration) {
        this.maxVelocity = maxVelocity;
        this.acceleration = acceleration;
        this.deceleration = deceleration;

        setMotion(0, 0);
    }

    public void setCoefficients(double maxVelocity, double acceleration, double deceleration) {
        this.maxVelocity = maxVelocity;
        this.acceleration = acceleration;
        this.deceleration = deceleration;
    }

    public void setMotion(double initialPosition, double finalPosition){
        this.initialPosition = initialPosition;
        this.finalPosition = finalPosition;
        double distance = Math.abs(finalPosition - initialPosition);
        if(distance == 0){
            return;
        }
        maxUsedVelocity = Math.min(maxVelocity, Math.sqrt((2 * distance * acceleration * deceleration) / (acceleration + deceleration)));
        T0 = maxUsedVelocity / acceleration;
        T2 = maxUsedVelocity / deceleration;
        T1 = Math.max(distance / maxUsedVelocity - (T0 + T2) / 2, 0);
        timer.reset();
    }

    private double position(double time){
        if(time <= T0){
            return acceleration * time * time / 2;
        }
        if(time <= T0 + T1){
            return acceleration * T0 * T0 / 2 + maxUsedVelocity * (time - T0);
        }
        if(time <= T0 + T1 + T2){
            return acceleration * T0 * T0 / 2 + maxUsedVelocity * (time - T0) - deceleration * (time - T0 - T1) * (time - T0 - T1) / 2;
        }
        return position(T0 + T1 + T2);
    }
    public double getPosition(){
        return initialPosition + Math.signum(finalPosition - initialPosition) * position(timer.seconds());
    }
    public double getRemainingTime(){
        return Math.max(T0 + T1 + T2 - timer.seconds(), 0);
    }
    public boolean motionEnded(){
        return Math.abs(finalPosition - getPosition()) <= 0.0001;
    }
    public void telemetry(Telemetry telemetry){
        telemetry.addData("Initial position: ", initialPosition);
        telemetry.addData("Final position: ", finalPosition);
        telemetry.addData("Max used velocity: ", maxUsedVelocity);
        telemetry.addData("Max Velocity: ", maxVelocity);
        telemetry.addData("Acceleration: ", acceleration);
        telemetry.addData("Deceleration: ", deceleration);
        telemetry.addData("Distance: ", Math.abs(finalPosition - initialPosition));
        telemetry.addData("Acceleration time: ", T0);
        telemetry.addData("Deceleration Time: ", T2);
        telemetry.addData("Gliding time: ", T1);
        telemetry.addData("Position: ", getPosition());
    }
}