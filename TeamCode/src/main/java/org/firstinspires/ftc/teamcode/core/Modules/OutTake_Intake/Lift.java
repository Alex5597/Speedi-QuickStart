package org.firstinspires.ftc.teamcode.core.Modules.OutTake_Intake;

import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LiftCorrectionCoefficients.gravityGain;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LiftCorrectionCoefficients.liftPIDCoefficients;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LiftPositions.liftCollect;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LiftPositions.liftHighBasket;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LiftPositions.liftHighChamber;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LiftPositions.liftLowBasket;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LiftPositions.liftLowChamber;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LiftPositions.liftScoreHighSpecimen;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.Modules.Module;
import org.firstinspires.ftc.teamcode.core.Util.Algorithm.LiftController;
import org.firstinspires.ftc.teamcode.core.Util.Hardware.CoolMotor;

public class Lift implements Module {
    LiftController PIDFController;
    CoolMotor liftMotor;

    public enum States {
        Collect,
        HighBasket,
        LowBasket,
        LowChamber,
        HighChamber,
        ScoreHighSpecimen,
        Reset,
        GOUP
    }

    public States state = States.Collect;
    double currentPosition = 0;

    public Lift(HardwareMap hardwareMap) {
        liftMotor = new CoolMotor(hardwareMap.get(DcMotorEx.class, "LM"), "motor lift");
        liftMotor.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.motor.setDirection(DcMotor.Direction.REVERSE);

        PIDFController = new LiftController(liftPIDCoefficients.p, liftPIDCoefficients.i, liftPIDCoefficients.d, gravityGain);
        PIDFController.setDesired_Position(0);
        PIDFController.setTolerance(10);
    }

    @Override
    public void update() {
        if (state != States.Reset) {
            currentPosition = liftMotor.motor.getCurrentPosition();
            PIDFController.setCoeff(liftPIDCoefficients.p, liftPIDCoefficients.i, liftPIDCoefficients.d, gravityGain);
            double power = PIDFController.getPow(currentPosition);
            liftMotor.setPowerForced(power);
        }
    }

    public void setState(States state) {
        this.state = state;
        switch (state) {
            case Collect:
                PIDFController.setDesired_Position(liftCollect);
                break;
            case LowBasket:
                PIDFController.setDesired_Position(liftLowBasket);
                break;
            case HighBasket:
                PIDFController.setDesired_Position(liftHighBasket);
                break;
            case LowChamber:
                PIDFController.setDesired_Position(liftLowChamber);
                break;
            case HighChamber:
                PIDFController.setDesired_Position(liftHighChamber);
                break;
            case ScoreHighSpecimen:
                PIDFController.setDesired_Position(liftScoreHighSpecimen);
                break;
            case GOUP:
                PIDFController.setDesired_Position(PIDFController.getTarget() + 150);
                break;
            case Reset:
                liftMotor.setPowerForced(-0.3);
                break;
        }
    }

    public void resetEnc() {
        liftMotor.setPowerForced(0);

        liftMotor.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        state = States.Collect;
        PIDFController.reset();
    }

    public boolean atTarget() {
        return PIDFController.atDesiredPos();
    }

    public double getCurrentPosition() {
        return currentPosition;
    }

    public double getPower(double currentPosition) {
        return PIDFController.getPow(currentPosition);
    }

    public double getTarget() {
        return PIDFController.getTarget();
    }

    public States getState() {
        return state;
    }
}
