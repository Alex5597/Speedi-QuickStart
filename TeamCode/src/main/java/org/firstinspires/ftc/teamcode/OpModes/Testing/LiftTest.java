package org.firstinspires.ftc.teamcode.OpModes.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;

@TeleOp
public class LiftTest extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap, new Pose(), telemetry, false);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a)
                robot.setAction(Robot.Actions.ScoreSpecimenHigh);
            if (gamepad1.b)
                robot.setAction(Robot.Actions.Collect);
            robot.drive.PinPointErrorTelemetry(true);
            telemetry.addData("Lift pos", robot.lift.getCurrentPosition());
            telemetry.addData("Lift power", robot.lift.getPower(robot.lift.getCurrentPosition()));
            telemetry.update();
            robot.update();
        }
    }
}
