package org.firstinspires.ftc.teamcode.core.Modules.OutTake_Intake;

import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.ColorValues.BlueValues;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.ColorValues.RedValues;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.ColorValues.TreshHold;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.ColorValues.YellowValues;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.Modules.Module;
import org.firstinspires.ftc.teamcode.core.Util.utils.Constants;

public class IntakeActive implements Module {
    ElapsedTime timerToStartCollectingAgain = new ElapsedTime();
    CRServo intakeLeft;
    CRServo intakeRight;
    ColorSensor intakeSensor;
    float hsvValues[] = {0F, 0F, 0F};

    public enum States {
        Collect,
        Wait,
        Score
    }

    public enum Color {
        Blue,
        Yellow,
        Red,
        None
    }

    public Color opposingAllianceColor = Color.Red;
    Color color = Color.None;
    public static States state = States.Wait;
    double power = Constants.IntakeActivePower.intakeWait, lastPower = -2;
    boolean shouldStartCollectingAgain = false;

    public IntakeActive(HardwareMap hardwareMap, Color allianceColor) {
        intakeLeft = hardwareMap.get(CRServo.class, "LI");
        intakeRight = hardwareMap.get(CRServo.class, "RI");
        intakeSensor = hardwareMap.get(ColorSensor.class, "IS");
        intakeLeft.setPower(power);
        intakeRight.setPower(power);
        if (allianceColor == Color.Red) {
            opposingAllianceColor = Color.Blue;
        } else {
            opposingAllianceColor = Color.Red;
        }
    }

    public void setState(States state) {
        this.state = state;
        lastPower = power;
        switch (state) {
            case Collect:
                power = Constants.IntakeActivePower.intakeCollect;
                color = Color.None;
                break;
            case Wait:
                power = Constants.IntakeActivePower.intakeWait;
                break;
            case Score:
                power = Constants.IntakeActivePower.intakeScore;
                break;
        }
    }

    @Override
    public void update() {
        if (power != lastPower) {
            intakeLeft.setPower(power);
            intakeRight.setPower(power);
        }
        if (!shouldStartCollectingAgain) {
            if (color == IntakeActive.Color.None)
                if (getColor() == opposingAllianceColor) {
                    setState(States.Score);
                    shouldStartCollectingAgain = true;
                    timerToStartCollectingAgain.reset();
                }
        } else if (timerToStartCollectingAgain.milliseconds() >= 700 && getColor() == Color.None) {
            setState(States.Collect);
            shouldStartCollectingAgain = false;
        }
    }

    public ColorSensor getColorSensor() {
        return intakeSensor;
    }

    public Color getColor() {
        if (Math.abs(hsvValues[0] - RedValues[0]) <= TreshHold[0]) {
            color = IntakeActive.Color.Red;
        } else if (Math.abs(hsvValues[0] - BlueValues[0]) <= TreshHold[0]) {
            color = IntakeActive.Color.Blue;
        } else if (Math.abs(hsvValues[0] - YellowValues[0]) <= TreshHold[0]) {
            color = IntakeActive.Color.Yellow;
        } else {
            color = IntakeActive.Color.None;
        }
        return color;
    }

    public void setAllianceColor(Color color) {
        if (color == Color.Red) {
            opposingAllianceColor = Color.Blue;
        } else {
            opposingAllianceColor = Color.Red;
        }
    }

    public Color getAllianceColor() {
        if (opposingAllianceColor == Color.Red) {
            return Color.Blue;
        } else {
            return Color.Red;
        }
    }
}
