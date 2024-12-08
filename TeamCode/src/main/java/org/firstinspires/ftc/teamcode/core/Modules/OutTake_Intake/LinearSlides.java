package org.firstinspires.ftc.teamcode.core.Modules.OutTake_Intake;

import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LinearSlidePositions.slidesExtendedPose;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LinearSlidePositions.slidesRetractedPose;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.core.Modules.Module;
import org.firstinspires.ftc.teamcode.core.Util.Hardware.BetterServo;

public class LinearSlides implements Module {

    BetterServo linkageServo;
    States state = States.Retracted;
    double target;

    public enum States {
        Extended,
        Retracted
    }

    public LinearSlides(HardwareMap hardwareMap) {
        linkageServo = new BetterServo(hardwareMap.get(Servo.class, "LS"), slidesRetractedPose);
        target = 0;
    }

    public void setState(States state) {
        this.state = state;
        switch (state) {
            case Extended:
                linkageServo.setPosition(slidesExtendedPose);
                target = slidesExtendedPose;
                break;
            case Retracted:
                linkageServo.setPosition(slidesRetractedPose);
                target = slidesRetractedPose;
                break;
        }
    }

    public void setTarget(double input){
        target = target + Range.scale(input, -1, 1,-0.03,0.03);
        target = Range.clip(target, 0, 0.3);
        linkageServo.setPosition(target);
    }

    @Override
    public void update() {
        linkageServo.update();
    }
}
