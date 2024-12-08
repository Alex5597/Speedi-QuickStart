package org.firstinspires.ftc.teamcode.core.Modules.Climb;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.Modules.Module;
import org.firstinspires.ftc.teamcode.core.Util.Hardware.CoolMotor;

public class ClimbModule implements Module {
    CoolMotor leftMotor, rightMotor;

    public enum States {
        Climbing,
        Waiting,
        ResetLeft,
        ResetRight
    }

    States state = States.Waiting;

    public ClimbModule(HardwareMap hardwareMap) {
        leftMotor = new CoolMotor(hardwareMap.get(DcMotorEx.class, "ASCL"), "motor catarare stanga");
        rightMotor = new CoolMotor(hardwareMap.get(DcMotorEx.class, "ASCR"), "motor catarare dreapta");
    }

    public void setState(States state) {
        this.state = state;
        switch (state) {
            case Waiting:
                leftMotor.setPowerForced(0);
                rightMotor.setPowerForced(0);
                break;
            case Climbing:
                leftMotor.setPowerForced(1);
                rightMotor.setPowerForced(1);
                break;
            case ResetLeft:
                leftMotor.setPowerForced(-1);
                break;
            case ResetRight:
                rightMotor.setPowerForced(-1);
                break;
        }
    }

    @Override
    public void update() {
        leftMotor.update();
        rightMotor.update();
    }
}
