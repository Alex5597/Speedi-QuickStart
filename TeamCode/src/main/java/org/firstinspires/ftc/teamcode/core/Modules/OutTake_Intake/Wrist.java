package org.firstinspires.ftc.teamcode.core.Modules.OutTake_Intake;

import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.WristPositions.wristCollectPose;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.WristPositions.wristParkPose;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.WristPositions.wristTransferPose;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.WristPositions.wristWaitPose;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.Modules.Module;
import org.firstinspires.ftc.teamcode.core.Util.Hardware.BetterServo;

public class Wrist implements Module {

    BetterServo wrist;
    States state = States.Transfer;

    public enum States {
        Collect,
        Transfer,
        Wait,
        Park
    }

    public Wrist(HardwareMap hardwareMap, boolean isAuto) {
        if(isAuto)
            wrist = new BetterServo(hardwareMap.get(Servo.class, "W"), wristWaitPose);
        else
            wrist = new BetterServo(hardwareMap.get(Servo.class, "W"));
    }
    public void setState(Wrist.States state) {
        this.state = state;
        switch (state) {
            case Collect:
                wrist.setPosition(wristCollectPose);
                break;
            case Transfer:
                wrist.setPosition(wristTransferPose);
                break;
            case Wait:
                wrist.setPosition(wristWaitPose);
                break;
            case Park:
                wrist.setPosition(wristParkPose);
                break;
        }
    }


    @Override
    public void update() {
        wrist.update();
    }
}
