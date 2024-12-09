package org.firstinspires.ftc.teamcode.core.Modules.OutTake_Intake;

import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.BucketPositions.bucketHoldPose;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.BucketPositions.bucketWaitPose;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.Modules.Module;
import org.firstinspires.ftc.teamcode.core.Util.Hardware.BetterServo;

public class Bucket implements Module {

    BetterServo servoBucket;
    ColorRangeSensor sensorBucket;

    public enum States {
        Hold,
        Wait
    }

    public static States state = States.Wait;
    IntakeActive intakeActiveSensor;

    public Bucket(HardwareMap hardwareMap, IntakeActive colorSensor) {
        intakeActiveSensor = colorSensor;
        servoBucket = new BetterServo(hardwareMap.get(Servo.class, "SB"), bucketWaitPose);
        sensorBucket = hardwareMap.get(ColorRangeSensor.class, "BS");

    }


    public void setState(States state) {
        Bucket.state = state;
        switch (state) {
            case Hold:
                servoBucket.setPosition(bucketHoldPose);
                break;
            case Wait:
                servoBucket.setPosition(bucketWaitPose);
                break;
        }
    }

    @Override
    public void update() {
        if (state == States.Wait)
            if (sensorBucket.getDistance(DistanceUnit.MM) <= 32)
                if (intakeActiveSensor.getColor() == IntakeActive.Color.None) {
                    setState(States.Hold);
                }
        servoBucket.update();
    }

    public double getDistanceFromSampleInBucket() {
        return sensorBucket.getDistance(DistanceUnit.MM);
    }
}
