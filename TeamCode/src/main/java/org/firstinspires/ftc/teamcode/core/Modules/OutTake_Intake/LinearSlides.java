package org.firstinspires.ftc.teamcode.core.Modules.OutTake_Intake;

import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LinearSlidePositions.slidesAuxPose;
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
    boolean canRetract = true;

    public enum States {
        Extended,
        Retracted,
        Aux
    }

    public LinearSlides(HardwareMap hardwareMap, boolean isAuto) {
        if (isAuto)
            linkageServo = new BetterServo(hardwareMap.get(Servo.class, "LS"), slidesRetractedPose);
        else
            linkageServo = new BetterServo(hardwareMap.get(Servo.class, "LS"));
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
            case Aux:
                linkageServo.setPosition(slidesAuxPose);
                target = slidesAuxPose;
                break;
        }
    }

    public void setTarget(double input) {
        if (input != 0)
            canRetract = false;
        target = target + Range.scale(input, -1, 1, -0.008, 0.008);
        target = Range.clip(target, 0, 0.3);
        linkageServo.setPosition(target);
    }

    public boolean canRetract() {
        return canRetract;
    }

    public double getTarget() {
        return target;
    }

    @Override
    public void update() {
        linkageServo.update();
    }
}
