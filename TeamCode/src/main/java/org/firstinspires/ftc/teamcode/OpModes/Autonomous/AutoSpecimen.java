package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.headignMultiplier;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.lateralMultiplier;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.Modules.Climb.ClimbModule;
import org.firstinspires.ftc.teamcode.core.Modules.OutTake_Intake.IntakeActive;
import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;

@Autonomous
public class AutoSpecimen extends LinearOpMode {
    Robot robot;
    ElapsedTime timer = new ElapsedTime();
    int state = 0;
    int i = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        robot = new Robot(hardwareMap, new Pose(), telemetry, true, IntakeActive.Color.Red);

        while (!isStarted() && !isStopRequested()) {
            robot.climb.setState(ClimbModule.States.Waiting);
            if (gamepad1.a)
                robot.climb.setState(ClimbModule.States.ResetLeft);
            if (gamepad1.b)
                robot.climb.setState(ClimbModule.States.ResetRight);
        }
        waitForStart();
        robot.setAction(Robot.Actions.ScoreSpecimenHigh);
        robot.drive.setMaxPower(0.5);
        robot.drive.setTargetPose(new Pose(-30, 77.5, 0));

        timer.reset();

        while (opModeIsActive()) {
            switch (state) {
                case 0:
                    if (robot.drive.isDone()) {
                        robot.setAction(Robot.Actions.ScoreSpecimenHigh);
                        state++;
                    }
                    break;
                case 1:
                    if (robot.isClawOpened()) {
                        robot.setAction(Robot.Actions.Collect);
                        robot.intakeSpecimen.close();
                        robot.drive.setTargetPose(new Pose(36, 50, Math.toRadians(90)));
                        state++;
                    }
                    break;
                case 2:
                    if (robot.drive.reachedTarget(5) && robot.drive.reachedHeading(10)) {
                        robot.drive.setTargetPose(new Pose(36, 130, Math.toRadians(180)));
                        robot.climb.setState(ClimbModule.States.Climbing);
                        timer.reset();
                        state++;
                    }
                    break;
                case 3:
                    if (robot.drive.reachedTarget(5)) {
                        robot.drive.setTargetPose(new Pose(70, 120, Math.toRadians(180)));
                        state++;
                    }
                    break;
                case 4:
                    if (robot.drive.reachedTarget(5)) {
                        robot.drive.setTargetPose(new Pose(70, 30, Math.toRadians(180)));
                        state++;
                    }
                    break;
                case 5:
                    if (robot.drive.reachedTarget(3.5)) {
                        robot.drive.setTargetPose(new Pose(60, 120, Math.toRadians(180)));
                        state++;
                    }
                    break;
                case 6:
                    if (robot.drive.reachedTarget(5)) {
                        robot.drive.setTargetPose(new Pose(90, 120, Math.toRadians(180)));
                        state++;
                    }
                    break;
                case 7:
                    if (robot.drive.reachedTarget(5)) {
                        robot.drive.setTargetPose(new Pose(100, 30, Math.toRadians(180)));
                        state++;
                    }
                    break;
                case 8:
                    if (robot.drive.reachedTarget(3.5)) {
                        robot.drive.setTargetPose(new Pose(100, 120, Math.toRadians(180)));
                        state++;
                    }
                    break;
                case 9:
                    if (robot.drive.reachedTarget(3.5)) {
                        robot.drive.setTargetPose(new Pose(113.5, 120, Math.toRadians(180)));
                        state++;
                    }
                    break;
                case 10:
                    if (robot.drive.reachedTarget(3)) {
                        robot.drive.setMaxPower(0.55);
                        robot.drive.setTargetPose(new Pose(113.5, 30, Math.toRadians(180)));
                        state++;
                    }
                    break;
                case 11:
                    if (robot.drive.reachedTarget(5)) {
                        robot.drive.setTargetPose(new Pose(50, 4.5, Math.toRadians(180)));
                        robot.drive.setMaxPower(0.5);
                        lateralMultiplier = 6;
                        state++;
                    }
                    break;
                case 12:
                    if (robot.drive.isDone()) {
                        robot.setAction(Robot.Actions.ScoreSpecimenHigh);
                        state++;
                        timer.reset();
                    }
                    break;
                case 13:
                    if (timer.milliseconds() >= 300) {
                        robot.drive.setTargetPose(new Pose(-22 + 5 * i, 77.5, 0));
                        robot.drive.setMaxPower(0.8);
                        headignMultiplier = 2;
                        state++;
                    }
                    break;
                case 14:
                    if (robot.drive.isDone()) {
                        robot.setAction(Robot.Actions.ScoreSpecimenHigh);
                        if (i == 2)
                            state = 17;
                        else
                            state++;
                    }
                    break;
                case 15:
                    if (robot.isClawOpened()) {
                        robot.setAction(Robot.Actions.Collect);
                        robot.intakeSpecimen.close();
                        state++;
                    }
                    break;
                case 16:
                    if (robot.isClawOpened()) {
                        headignMultiplier = 2;
                        robot.drive.setTargetPose(new Pose(50, 5, Math.toRadians(180)));
                        robot.drive.setMaxPower(0.8);
                        i++;
                        state = 12;
                    }
                    break;
                case 17:
                    if (robot.isClawOpened()) {
                        robot.drive.setTargetPose(new Pose(50, 10, Math.toRadians(180)));
                        robot.setAction(Robot.Actions.Collect);
                        robot.intakeSpecimen.open();
                        state++;
                    }
                    break;
            }
            if (timer.seconds() >= 3.5)
                robot.climb.setState(ClimbModule.States.Waiting);
            robot.update();
        }
    }
}
