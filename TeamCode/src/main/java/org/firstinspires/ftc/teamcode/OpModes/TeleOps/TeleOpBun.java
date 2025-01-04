package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.IntakeActivePower.intakeScore;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LinearSlidePositions.slidesRetractedPose;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.Modules.Climb.ClimbModule;
import org.firstinspires.ftc.teamcode.core.Modules.OutTake_Intake.Bucket;
import org.firstinspires.ftc.teamcode.core.Modules.OutTake_Intake.IntakeActive;
import org.firstinspires.ftc.teamcode.core.Modules.OutTake_Intake.Lift;
import org.firstinspires.ftc.teamcode.core.Modules.OutTake_Intake.LinearSlides;
import org.firstinspires.ftc.teamcode.core.Modules.OutTake_Intake.Wrist;
import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;


@TeleOp
public class TeleOpBun extends LinearOpMode {

    Robot robot;
    ElapsedTime timerTrigger = new ElapsedTime();
    ElapsedTime timerBumper = new ElapsedTime();
    ElapsedTime timerSpecimenHigh = new ElapsedTime();
    ElapsedTime timerSpecimenLow = new ElapsedTime();
    ElapsedTime timerSampleHigh = new ElapsedTime();
    ElapsedTime timerSampleLow = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timerLift = new ElapsedTime();


    public enum States {
        Specimens,
        Samples
    }

    public static States state = TeleOpBun.States.Specimens;
    boolean canRetract = false;
    boolean shouldRetract = false;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap, new Pose(), telemetry, false, IntakeActive.Color.Red);
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.a)
                robot.intakeSample.setAllianceColor(IntakeActive.Color.Red);
            else
                robot.intakeSample.setAllianceColor(IntakeActive.Color.Blue);
            telemetry.addData("Current selected color for our alliance", robot.intakeSample.getAllianceColor());
            telemetry.update();
        }
        state = TeleOpBun.States.Specimens;
        waitForStart();
        telemetry.addLine("OK");
        telemetry.update();
        while (opModeIsActive()) {
            //Driver 1 Controls

            if (gamepad1.dpad_up)
                robot.drive.resetPosition(new Pose());
            robot.drive.motors.driveFieldCentric(gamepad1, robot.drive.getCurrentPos().getHeading());

            //Climb
            robot.climb.setState(ClimbModule.States.Waiting);
            if (gamepad1.right_bumper) {
                robot.climb.setState(ClimbModule.States.Climbing);
            }
            if (gamepad1.left_trigger >= 0.4) {
                robot.climb.setState(ClimbModule.States.ResetLeft);
            }
            if (gamepad1.right_trigger >= 0.4) {
                robot.climb.setState(ClimbModule.States.ResetRight);
            }

            //Driver 2 Controls

            //Wrist
            if (gamepad2.dpad_down) {
                robot.wrist.setState(Wrist.States.Collect);
            }
            if (gamepad2.dpad_left) {
                robot.wrist.setState(Wrist.States.Wait);
            }
            if (gamepad2.dpad_up) {
                robot.wrist.setState(Wrist.States.Transfer);
            }

            //Intake Sample
            if (gamepad2.right_trigger >= 0.4 && timerTrigger.milliseconds() >= 500) {
                if (IntakeActive.state == IntakeActive.States.Collect) {
                    robot.intakeSample.setState(IntakeActive.States.Score);
                } else {
                    robot.intakeSample.setState(IntakeActive.States.Collect);
                }
                timerTrigger.reset();
            }
            if (gamepad2.left_trigger >= 0.4) {
                robot.intakeSample.setState(IntakeActive.States.Wait);
            }
            if (gamepad2.b) {
                robot.setAction(Robot.Actions.Collect);
            }
            if (robot.intakeSample.getPower() == intakeScore) {
                gamepad2.rumble(100);
            }
            if (gamepad1.dpad_left)
                robot.lift.setState(Lift.States.Reset);
            if (gamepad1.dpad_right)
                robot.lift.resetEnc();
            switch (state) {
                case Specimens:

                    //Lift
                    if (gamepad2.a && timerSpecimenHigh.milliseconds() >= 500) {
                        robot.setAction(Robot.Actions.ScoreSpecimenHigh);
                        timerSpecimenHigh.reset();
                    }
                    if (gamepad2.x && timerSpecimenLow.milliseconds() >= 500) {
                        robot.setAction(Robot.Actions.ScoreSpecimenLow);
                        timerSpecimenLow.reset();
                    }
                    //Intake Specimen
                    if (gamepad2.left_bumper) {
                        robot.intakeSpecimen.open();
                    }
                    if (gamepad2.right_bumper) {
                        robot.intakeSpecimen.close();
                    }
                    break;
                case Samples:

                    //Bucket
                    if (gamepad2.right_bumper && timerBumper.milliseconds() >= 500) {
                        if (Bucket.state == Bucket.States.Hold) {
                            robot.bucket.setState(Bucket.States.Wait);
                        } else {
                            robot.bucket.setState(Bucket.States.Hold);
                        }
                        timerBumper.reset();
                    }
                    if (gamepad2.left_bumper) {
                        robot.bucket.setState(Bucket.States.Wait);
                    }
                    if (gamepad2.a && timerSampleHigh.milliseconds() >= 500) {
                        robot.setAction(Robot.Actions.ScoreSampleHigh);
                        timerSampleHigh.reset();
                    }
                    if (gamepad2.x && timerSampleLow.milliseconds() >= 500) {
                        robot.setAction(Robot.Actions.ScoreSampleLow);
                        timerSampleLow.reset();
                    }
                    break;
            }
            if (gamepad2.left_stick_button) {
                robot.setAction(Robot.Actions.SlidesRetracted);
                if (robot.intakeSample.getColor() == IntakeActive.Color.None)
                    canRetract = true;
            }
            if (gamepad2.right_stick_button) {
                robot.setAction(Robot.Actions.SlidesExtended);
                if (robot.intakeSample.getColor() == IntakeActive.Color.None)
                    canRetract = true;
            }
            if (!shouldRetract)
                if (timer.milliseconds() >= 700 && canRetract && robot.intakeSample.canRetract()) {
                    timer.reset();
                    shouldRetract = true;
                }
            if (shouldRetract && timer.milliseconds() >= 300) {
                if (canRetract && robot.intakeSample.canRetract()) {
                    shouldRetract = false;
                    robot.setAction(Robot.Actions.SlidesRetracted);
                    gamepad1.rumble(500);
                    canRetract = false;
                } else
                    shouldRetract = false;
            }
            if (gamepad2.right_stick_y != 0)
                robot.slides.setTarget(robot.drive.motors.smoothControls(-gamepad2.right_stick_y));
            if (gamepad1.left_bumper && timerLift.milliseconds() >= 200) {
                robot.setAction(Robot.Actions.GOUP);
                timerLift.reset();
            }

            //Update
            robot.update();
            telemetry.addData("Culoare citita", robot.intakeSample.getColor());
            telemetry.update();
        }

    }
}
