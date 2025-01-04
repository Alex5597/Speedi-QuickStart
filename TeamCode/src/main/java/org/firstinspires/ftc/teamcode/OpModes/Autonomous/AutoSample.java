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
import org.firstinspires.ftc.teamcode.core.Modules.OutTake_Intake.Bucket;
import org.firstinspires.ftc.teamcode.core.Modules.OutTake_Intake.IntakeActive;
import org.firstinspires.ftc.teamcode.core.Modules.OutTake_Intake.LinearSlides;
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

        while (!isStarted() || isStopRequested()) {
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
        robot.drive.localizer.odo.setPosition(new Pose2D(DistanceUnit.CM, -20, 0, AngleUnit.RADIANS, 0));
        telemetry.addLine("GATA");
        telemetry.update();

        robot.drive.setMaxPower(0.4);
        waitForStart();
        robot.setAction(Robot.Actions.ScoreSpecimenHigh);

        robot.drive.setTargetPose(new Pose(3, 77.5, 0));


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
                        robot.drive.setTargetPose(new Pose(-97, 20, Math.toRadians(180)), new Pose(2, 2, 1.5));
                        state++;
                    }
                    break;
                case 2:
                    if (robot.drive.isDone()) {
                        robot.climb.setState(ClimbModule.States.AutoClimb, 4);
                        robot.setAction(Robot.Actions.InstantPutWristDown);
                        timer.reset();
                        state++;
                    }
                    break;
                case 3:
                    if (timer.milliseconds() >= 100) {
                        robot.drive.setMaxPower(0.005);
                        robot.drive.setTargetPose(new Pose(-97, 92, Math.toRadians(180)), new Pose(2, 2, 1.5));
                        timer.reset();
                        state++;
                    }
                    break;
                case 4:
                    if (robot.intakeSample.getColor() != IntakeActive.Color.None && timer.milliseconds() >= 900) {
                        robot.setAction(Robot.Actions.SlidesRetracted);
                        headignMultiplier = 2;
                        robot.drive.setMaxPower(1);
                        robot.drive.setTargetPose(new Pose(-121, 22, Math.toRadians(135)), new Pose(2, 2, 4));
                        state++;
                    }
                    break;
                case 5:
                    if (robot.drive.isDone()) {
                        timer.reset();
                        state = 55;
                    }
                    break;
                case 55:
                    if (timer.milliseconds() >= 600) {
                        robot.intakeSample.setState(IntakeActive.States.Score);
                        state = 6;
                    }
                    break;
                case 6:
                    if (robot.bucket.getState() == Bucket.States.Hold) {
                        timer.reset();
                        state = 65;
                    }
                    break;
                case 65:
                    if (timer.milliseconds() >= 500) {
                        robot.setAction(Robot.Actions.ScoreSampleHigh);
                        timer.reset();
                        state = 7;
                    }
                    break;
                case 7:
                    if (timer.milliseconds() >= 2000) {
                        robot.setAction(Robot.Actions.ScoreSampleHigh);
                        timer.reset();
                        state++;
                    }
                    break;
                case 8:
                    if (robot.bucket.getState() == Bucket.States.Wait && timer.milliseconds() >= 500) {
                        robot.setAction(Robot.Actions.Collect);
                        headignMultiplier = 4;
                        robot.drive.setMaxPower(1);
                        robot.drive.setTargetPose(new Pose(-122, 20, Math.toRadians(180)), new Pose(2, 2, 1.5));
                        state++;
                    }
                    break;
                case 9:
                    if (robot.drive.isDone() && robot.drive.getHeadingError() <= Math.toRadians(2)) {
                        robot.setAction(Robot.Actions.InstantPutWristDown);
                        timer.reset();
                        state++;
                        //So we have no warning here
                    }
                    break;
                case 10:
                    if (timer.milliseconds() >= 100) {
                        robot.drive.setMaxPower(0.005);
                        robot.drive.setTargetPose(new Pose(-122, 95, Math.toRadians(180)), new Pose(2, 2, 1.5));
                        timer.reset();
                        state++;
                    }
                    break;
                case 11:
                    if (robot.intakeSample.getColor() != IntakeActive.Color.None && timer.milliseconds() >= 900) {
                        robot.setAction(Robot.Actions.SlidesRetracted);
                        headignMultiplier = 2;
                        robot.drive.setMaxPower(1);
                        robot.drive.setTargetPose(new Pose(-122, 21, Math.toRadians(135)), new Pose(2, 2, 4));
                        state++;
                        //So we have no warning here

                    }
                    break;
                case 12:
                    if (robot.drive.isDone()) {
                        timer.reset();
                        state++;
                        //So we have no warning here

                    }
                    break;
                case 13:
                    if (timer.milliseconds() >= 600) {
                        robot.intakeSample.setState(IntakeActive.States.Score);
                        state++;
                    }
                    break;
                case 14:
                    if (robot.bucket.getState() == Bucket.States.Hold) {
                        timer.reset();
                        state = 145;
                        //So we have no warning here

                    }
                    break;
                case 145:
                    if (timer.milliseconds() >= 500) {
                        robot.setAction(Robot.Actions.ScoreSampleHigh);
                        timer.reset();
                        state = 15;
                    }
                case 15:
                    if (timer.milliseconds() >= 2000) {
                        robot.setAction(Robot.Actions.ScoreSampleHigh);
                        timer.reset();
                        state++;
                        //So we have no warning here

                    }
                    break;
                case 16:
                    if (robot.bucket.getState() == Bucket.States.Wait && timer.milliseconds() >= 500) {
                        robot.setAction(Robot.Actions.Collect);
                        robot.drive.setMaxPower(1);
                        headignMultiplier = 2;
                        robot.drive.setTargetPose(new Pose(-105, 60, Math.toRadians(-120)));
                        state++;
                        //So we have no warning here

                    }
                    break;
                case 17:
                    if (robot.drive.isDone()) {
                        robot.setAction(Robot.Actions.InstantPutWristDown);
                        timer.reset();
                        state = 175;
                        //ndaisnf
                    }
                    break;
                case 175:
                    if (timer.milliseconds() >= 100) {
                        robot.drive.setMaxPower(0.005);
                        robot.drive.setTargetPose(new Pose(-105, 70, Math.toRadians(-120)), new Pose(2, 2, 3));
                        state = 18;
                        timer.reset();
                    }
                case 18:
                    if (robot.intakeSample.getColor() != IntakeActive.Color.None && timer.milliseconds() >= 900) {
                        robot.setAction(Robot.Actions.SlidesRetracted);
                        headignMultiplier = 2;
                        robot.drive.setMaxPower(1);
                        robot.drive.setTargetPose(new Pose(-122, 21, Math.toRadians(135)), new Pose(2, 2, 4));
                        state++;
                    }
                    break;
                case 19:
                    if (robot.drive.isDone()) {
                        timer.reset();
                        state++;
                        //So we have no warning here

                    }
                    break;
                case 20:
                    if (timer.milliseconds() >= 600) {
                        robot.intakeSample.setState(IntakeActive.States.Score);
                        state++;
                    }
                    break;
                case 21:
                    if (robot.bucket.getState() == Bucket.States.Hold) {
                        timer.reset();
                        state = 215;
                        //So we have no warning here

                    }
                    break;
                case 215:
                    if (timer.milliseconds() >= 500) {
                        robot.setAction(Robot.Actions.ScoreSampleHigh);
                        timer.reset();
                        state = 22;
                    }
                    break;
                case 22:
                    if (timer.milliseconds() >= 2000) {
                        robot.setAction(Robot.Actions.ScoreSampleHigh);
                        timer.reset();
                        state++;
                        //So we have no warning here

                    }
                    break;
                case 23:
                    if (robot.bucket.getState() == Bucket.States.Wait && timer.milliseconds() >= 500) {
                        robot.setAction(Robot.Actions.Collect);
                        robot.climb.setState(ClimbModule.States.Climbing, 1.7);
                        robot.drive.setTargetPose(new Pose(-100, 135, Math.toRadians(90)));
                        state++;
                        //So we have no warning here s

                    }
                    break;
                case 24:
                    if (robot.drive.reachedTarget(5)) {
                        robot.slides.setState(LinearSlides.States.Extended);
                        robot.drive.setTargetPose(new Pose(-60, 135, Math.toRadians(90)));
                        state++;
                    }
                    break;
                case 25:
                    if (robot.drive.isDone()) {
                        state++;
                    }
                    break;
            }
            robot.update();
            robot.drive.currentPosTelemetry(true);
            robot.drive.errorTelemetry(true);
            telemetry.addData("Current state", state);
            telemetry.update();
        }
    }
}
