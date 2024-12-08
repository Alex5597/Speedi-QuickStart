package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.Modules.OutTake_Intake.IntakeActive;
import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;

@Autonomous
public class AutoSample extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime();
    Robot robot;
    int state = 0, i = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        robot = new Robot(hardwareMap, new Pose(), telemetry, true, IntakeActive.Color.Red);

        waitForStart();
        robot.intakeSample.setState(IntakeActive.States.Collect);
        robot.drive.setTargetPose(new Pose(-90, 30, Math.toRadians(135)));
        while (opModeIsActive()) {
            switch (state) {

                case 0:
                    if ((robot.drive.reachedTarget(5) && robot.drive.reachedHeading(5)) || robot.drive.isStuck()) {
                        robot.drive.setTargetPose(new Pose(-94, 12.8, Math.toRadians(135)));
                        if (i == 2)
                            robot.drive.setTargetPose(new Pose(-91, 8, Math.toRadians(135)));
                        if (i == 3)
                            robot.drive.setTargetPose(new Pose(-93, 7, Math.toRadians(135)));
                        robot.setAction(Robot.Actions.ScoreSpecimenHigh);
                        state++;
                    }
                    break;
                case 1:
                    if (robot.drive.isDone() && robot.lift.atTarget()) {
                        robot.setAction(Robot.Actions.ScoreSpecimenHigh);
                        timer.reset();
                        state++;
                    }
                    break;
                case 2:
                    if (timer.milliseconds() >= 550) {
                        if (i == 3) {
                            state = 6;
                            robot.setAction(Robot.Actions.Collect);
                        } else {
                            robot.drive.setTargetPose(new Pose(-80, 15, Math.toRadians(135)));
                            robot.setAction(Robot.Actions.Collect);
                            timer.reset();
                            state++;
                        }
                    }
                    break;
                case 3:
                    if (robot.lift.atTarget()) {
                        if (i == 0) {
                            robot.drive.setTargetPose(new Pose(-45, 89, Math.toRadians(-90)));
                        } else if (i == 1) {
                            robot.drive.setTargetPose(new Pose(-70, 91, Math.toRadians(-90)));
                        } else if (i == 2) {
                            robot.drive.setTargetPose(new Pose(-110, 85, Math.toRadians(-90)));
                        }
                        state++;
                    }
                    break;
                case 4:
                    if (robot.drive.isDone()) {
                        if (i == 0) {
                            robot.drive.setTargetPose(new Pose(-80, 89, Math.toRadians(-90)));
                        } else if (i == 1) {
                            robot.drive.setTargetPose(new Pose(-100, 90, Math.toRadians(-90)));
                        } else if (i == 2) {
                            robot.drive.setTargetPose(new Pose(-110, 85, Math.toRadians(-90)));
                        }
                        i++;
                        state++;
                    }
                    break;
                case 5:
                    if (robot.drive.isDone()) {
                        robot.drive.setTargetPose(new Pose(-90, 30, Math.toRadians(135)));
                        state = 0;
                    }
                    break;
                case 6:
                    if (robot.lift.atTarget()) {
                        robot.drive.setTargetPose(new Pose(-45, 100, Math.toRadians(180)));
                        state++;
                    }
                    break;
            }
            robot.update();
            telemetry.addData("State", state);
            telemetry.addData("Current pose", robot.lift.getCurrentPosition());
            telemetry.addData("Target pose", robot.lift.getTarget());
            telemetry.update();
        }
    }
}
