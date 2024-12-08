package org.firstinspires.ftc.teamcode.core.Util.Algorithm;

import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LiftCorrectionCoefficients.gravityGain;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.qualcomm.robotcore.util.Range;

public class LiftController {
    PIDFController Lift_Controller;
    ElevatorFeedforward Lift_FeedForward;
    private double desired_Position_Arm = 0, current_pos = 0;
    boolean disabled = false;

    public LiftController(double kP, double kI, double kD, double kF) {
        Lift_Controller = new PIDFController(kP, kI, kD, 0);
        Lift_FeedForward = new ElevatorFeedforward(0, kF, 0, 0);

        Lift_Controller.setTolerance(0);
    }

    public double getPow(double curr_pos_Lift) {
        current_pos = curr_pos_Lift;
        double PID_Control = Lift_Controller.calculate(curr_pos_Lift, desired_Position_Arm);
        double FeedForward = Lift_FeedForward.calculate(0, 0);

        if (disabled) return 0;
        return Range.clip(PID_Control + FeedForward, -1, 1);
    }

    public void setDesired_Position(double desired_Position_Arm) {
        this.desired_Position_Arm = desired_Position_Arm;
        Lift_Controller.setSetPoint(desired_Position_Arm);
    }

    public double getTarget() {
        return desired_Position_Arm;
    }

    public boolean atDesiredPos() {
        return Math.abs(desired_Position_Arm - current_pos) <= 20;
    }

    public void disable() {
        disabled = true;
    }

    public void enable() {
        disabled = false;
        Lift_Controller.reset();
    }

    public void setTolerance(double tolerance) {
        Lift_Controller.setTolerance(tolerance);
    }

    public void reset() {
        Lift_Controller.reset();
        Lift_FeedForward = new ElevatorFeedforward(0, gravityGain, 0, 0);
    }

    public void setCoeff(double kP, double kI, double kD, double kF) {
        Lift_Controller.setPIDF(kP, kI, kD, 0);
        Lift_FeedForward = new ElevatorFeedforward(0, kF, 0, 0);
    }
}
