package org.firstinspires.ftc.teamcode.core.Modules.OutTake_Intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.Modules.Module;
import org.firstinspires.ftc.teamcode.core.Util.Hardware.BetterServo;
import org.firstinspires.ftc.teamcode.core.Util.utils.Constants;

public class Arm implements Module {
    BetterServo armServo;

    public enum States {
        Collect,
        Place,
    }

    States state = States.Collect;

    public Arm(HardwareMap hardwareMap) {
        armServo = new BetterServo(hardwareMap.get(Servo.class, "A"), 5, 2.5, 10, Constants.ArmPoses.armCollectPose);
    }

    public void setState(States state) {
        this.state = state;
        switch (state) {
            case Collect:
                armServo.setPosition(Constants.ArmPoses.armCollectPose);
                break;
            case Place:
                armServo.setPosition(Constants.ArmPoses.armScorePose);
                break;
        }
    }

    @Override
    public void update() {
        armServo.update();
    }
}
