package org.firstinspires.ftc.teamcode.core.Util.Algorithm;


import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.qualcomm.robotcore.util.Range;

public class LiftController {
    PIDFController Lift_Controller;
    ElevatorFeedforward Lift_FeedForward;
    private double desired_Position_Arm = 0, current_pos = 0;
    boolean disabled = false;
    private double gravityGain = 0;
    private double maxPower = 1;
    private double tolerance = 0;

    public LiftController(double kP, double kI, double kD, double kF) {
        gravityGain = kF;
        Lift_Controller = new PIDFController(kP, kI, kD, 0);
        Lift_FeedForward = new ElevatorFeedforward(0, kF, 0, 0);

        Lift_Controller.setTolerance(0);
    }

    public double getPow(double curr_pos_Lift) {
        if (disabled) return 0;

        current_pos = curr_pos_Lift;
        double PID_Control = Lift_Controller.calculate(curr_pos_Lift, desired_Position_Arm);
        double FeedForward = Lift_FeedForward.calculate(0, 0);

        return Range.clip(PID_Control + FeedForward, -maxPower, maxPower);
    }

    public void setDesired_Position(double desired_Position_Arm) {
        this.desired_Position_Arm = desired_Position_Arm;
        Lift_Controller.setSetPoint(desired_Position_Arm);
    }

    public double getTarget() {
        return desired_Position_Arm;
    }

    public boolean atDesiredPos() {
        return Math.abs(desired_Position_Arm - current_pos) <= tolerance;
    }

    public void disable() {
        disabled = true;
    }

    public boolean isDisabled() {
        return disabled;
    }

    public void enable() {
        disabled = false;
        Lift_Controller.reset();
    }

    public void setTolerance(double tolerance) {
        Lift_Controller.setTolerance(tolerance);
        this.tolerance = tolerance;
    }

    public void reset() {
        disabled = false;
        Lift_Controller.reset();
        Lift_FeedForward = new ElevatorFeedforward(0, gravityGain, 0, 0);
    }

    public void setCoeff(double kP, double kI, double kD, double kF, double maxPower) {
        this.maxPower = Math.abs(maxPower);
        gravityGain = kF;
        Lift_Controller.setPIDF(kP, kI, kD, 0);
        Lift_FeedForward = new ElevatorFeedforward(0, kF, 0, 0);
    }
}

