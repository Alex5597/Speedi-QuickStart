package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.forwardMultiplier;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.headignMultiplier;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.lateralMultiplier;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.core.Modules.Climb.ClimbModule;
import org.firstinspires.ftc.teamcode.core.Modules.OutTake_Intake.IntakeActive;
import org.firstinspires.ftc.teamcode.core.Modules.OutTake_Intake.LinearSlides;
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
            if (gamepad1.y)
                break;
        }
        robot.drive.localizer.odo.recalibrateIMU();
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        robot.drive.localizer.odo.setPosition(new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.RADIANS, 0));
        telemetry.addLine("GATA");
        telemetry.update();

        waitForStart();
        robot.setAction(Robot.Actions.ScoreSpecimenHigh);
        robot.drive.setMaxPower(0.4);
        robot.drive.setTargetPose(new Pose(-27, 77.5, 0));

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
                        headignMultiplier = 3;
                        robot.drive.setTargetPose(new Pose(39, 50, Math.toRadians(90)));
                        state++;
                    }
                    break;
                case 2:
                    if (robot.drive.reachedTarget(5) && robot.drive.reachedHeading(10)) {
                        headignMultiplier = 2.5;
                        forwardMultiplier = 1.5;
                        robot.drive.setTargetPose(new Pose(45, 130, Math.toRadians(180)));
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
                        //robot.slides.setState(LinearSlides.States.Extended);
                        robot.drive.setTargetPose(new Pose(70, 30, Math.toRadians(180)));
                        state++;
                    }
                    break;
                case 5:
                    if (robot.drive.reachedTarget(4)) {
                        robot.slides.setState(LinearSlides.States.Retracted);
                        robot.drive.setTargetPose(new Pose(80, 120, Math.toRadians(180)));
                        state++;
                    }
                    break;
                case 6:
                    if (robot.drive.reachedTarget(6)) {
                        robot.drive.setTargetPose(new Pose(100, 120, Math.toRadians(180)));
                        state++;
                    }
                    break;
                case 7:
                    if (robot.drive.reachedTarget(5)) {
                        robot.slides.setState(LinearSlides.States.Extended);
                        robot.drive.setTargetPose(new Pose(95, 30, Math.toRadians(180)));
                        state++;
                    }
                    break;
                case 8:
                    if (robot.drive.reachedTarget(6)) {
                        robot.slides.setState(LinearSlides.States.Retracted);
                        robot.drive.setTargetPose(new Pose(95, 120, Math.toRadians(180)));
                        state++;
                    }
                    break;
                case 9:
                    if (robot.drive.reachedTarget(3.5) && robot.drive.reachedHeading(5)) {
                        robot.drive.setTargetPose(new Pose(117.5, 120, Math.toRadians(180)), new Pose(5, 5, 5));

                        state++;
                    }
                    break;
                case 10:
                    if (robot.drive.isDone()) {
                        robot.drive.setMaxPower(0.25);
                        robot.slides.setState(LinearSlides.States.Extended);
                        robot.drive.setTargetPose(new Pose(117.5, 25, Math.toRadians(180)));
                        timer.reset();
                        state++;
                    }
                    break;
                case 11:
                    if (robot.drive.reachedTarget(5) || timer.seconds() >= 2) {
                        robot.slides.setState(LinearSlides.States.Retracted);
                        robot.drive.setTargetPose(new Pose(50, 2.5, Math.toRadians(180)), new Pose(3, 1, 3));
                        robot.drive.setMaxPower(0.3);
                        lateralMultiplier = 5;
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
                    if (timer.milliseconds() >= 500) {
                        lateralMultiplier = 2;
                        headignMultiplier = 4;
                        robot.drive.setMaxPower(0.9);
                        robot.drive.setTargetPose(new Pose(-25 + 3.5 * i, 77, 0), new Pose(3.5, 2, 5));
                        state++;
                    }
                    break;
                case 14:
                    if (robot.drive.isDone()) {
                        robot.setAction(Robot.Actions.ScoreSpecimenHigh);
                        if (i == 2)
                            state = 16;
                        else
                            state++;
                        timer.reset();
                    }
                    break;
                case 15:
                    if (robot.isClawOpened() && timer.milliseconds() >= 200) {
                        robot.setAction(Robot.Actions.Collect);
                        headignMultiplier = 3;
                        lateralMultiplier = 2;
                        robot.drive.setTargetPose(new Pose(50, 2, Math.toRadians(180)), new Pose(2.5, 1.5, 4.5));
                        robot.drive.setMaxPower(0.7);
                        i++;
                        state = 12;
                    }
                    break;
                case 16:
                    if (robot.isClawOpened()) {
                        lateralMultiplier = 3;
                        robot.drive.setTargetPose(new Pose(50, 40, 0));
                        robot.slides.setState(LinearSlides.States.Extended);
                        robot.setAction(Robot.Actions.Collect);
                        state++;
                    }
                    break;
            }
            if (timer.seconds() >= 3.5)
                robot.climb.setState(ClimbModule.States.Waiting);
            robot.update();
            telemetry.addData("Velocity", robot.drive.localizer.getVelocity());
            telemetry.addData("Stuck", robot.drive.robotIsStuck);
            telemetry.addData("Current state", state);
            telemetry.update();
        }
    }
}
