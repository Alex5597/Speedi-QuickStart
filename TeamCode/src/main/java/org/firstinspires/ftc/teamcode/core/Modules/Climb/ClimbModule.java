package org.firstinspires.ftc.teamcode.core.Modules.Climb;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.Modules.Module;
import org.firstinspires.ftc.teamcode.core.Util.Hardware.CoolMotor;

public class ClimbModule implements Module {
    ElapsedTime timer = new ElapsedTime();
    Telemetry telemetry;
    CoolMotor leftMotor, rightMotor;

    public enum States {
        Climbing,
        Waiting,
        ResetLeft,
        ResetRight,
        AutoClimb
    }

    States state = States.Waiting;
    double timeToRaise = 0;
    boolean done = true;

    public ClimbModule(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
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

    public void setState(States state, double timeToRaiseInSeconds) {
        this.state = state;
        switch (state) {
            case Waiting:
                leftMotor.setPowerForced(0);
                rightMotor.setPowerForced(0);
                break;
            case Climbing:
                timeToRaise = timeToRaiseInSeconds;
                timer.reset();
                done = false;
                leftMotor.setPowerForced(1);
                rightMotor.setPowerForced(1);
                break;
            case ResetLeft:
                leftMotor.setPowerForced(-1);
                break;
            case ResetRight:
                rightMotor.setPowerForced(-1);
                break;
            case AutoClimb:
                timeToRaise = timeToRaiseInSeconds;
                timer.reset();
                done = false;
                leftMotor.setPowerForced(-1);
                rightMotor.setPowerForced(-1);
                break;
        }
    }

    @Override
    public void update() {
        if (timer.seconds() >= timeToRaise && !done) {
            setState(States.Waiting);
            done = true;
        }
        leftMotor.update();
        rightMotor.update();
        telemetry.addData("Timp", timer.seconds());
        telemetry.addData("Timp target", timeToRaise);
    }
}
