package org.firstinspires.ftc.teamcode.core.Modules.OutTake_Intake;

import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.IntakePassivePositions.intakePassiveClosePose;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.IntakePassivePositions.intakePassiveOpenPose;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.Modules.Module;
import org.firstinspires.ftc.teamcode.core.Util.Hardware.BetterServo;

public class IntakePassive implements Module {
    public BetterServo claw;
    double servoPosition;

    public IntakePassive(HardwareMap hardwareMap, double servoPosition) {
        claw = new BetterServo(hardwareMap.get(Servo.class, "IP"), servoPosition);
        this.servoPosition = servoPosition;
    }

    public void open() {
        claw.setPosition(intakePassiveOpenPose);
        servoPosition = intakePassiveOpenPose;
    }

    public void close() {
        claw.setPosition(intakePassiveClosePose);
        servoPosition = intakePassiveClosePose;
    }

    @Override
    public void update() {
        claw.update();
    }
}
