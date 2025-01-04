package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

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
import org.firstinspires.ftc.teamcode.core.Modules.OutTake_Intake.Wrist;
import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.core.Util.Algorithm.SplineGenerator.BezierSpline;
import org.firstinspires.ftc.teamcode.core.Util.Algorithm.SplineGenerator.CubicBezierCurve;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;
import org.firstinspires.ftc.teamcode.core.Util.Math.Vector;
import org.firstinspires.ftc.teamcode.core.Util.utils.Constants;

@Autonomous
public class CrazyAuto extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime();
    Robot robot;
    int state = 0;
    int i = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap, new Pose(), telemetry, true, IntakeActive.Color.Blue);
        BezierSpline splineFromChamberToFirstSample = new BezierSpline(Math.toRadians(120), new CubicBezierCurve(new Vector(-20, 77.5), new Vector(-20, 60), new Vector(15, 45), new Vector(31.5, 75)));

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
        robot.drive.localizer.odo.setPosition(new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.RADIANS, 0));

        waitForStart();
        robot.setAction(Robot.Actions.ScoreSpecimenHigh);
        robot.drive.setMaxPower(0.85);
        robot.drive.setTargetPose(new Pose(-35, 77.5, 0));

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
                        //robot.drive.setSpline_withSlowerHeadingChange(splineFromChamberToFirstSample, 1);
                        headignMultiplier = 0.3;
                        robot.drive.setTargetPose(new Pose(21, 78, Math.toRadians(130)), new Pose(2, 3, 6));
                        timer.reset();
                        state++;
                    }
                    break;
                case 2:
                    if (timer.milliseconds() >= 1000) {
                        robot.slides.setState(LinearSlides.States.Extended);
                        robot.wrist.setState(Wrist.States.Collect);
                        state++;
                    }
                    break;
                case 3:
                    if (robot.drive.isDone()) {
                        robot.drive.setTargetPose(new Pose(21, 40, Math.toRadians(70)), new Pose(6, 6, 3));
                        state++;
                    }
                    break;
                case 4:
                    if (robot.drive.isDone()) {
                        robot.wrist.setState(Wrist.States.Wait);
                        robot.drive.setTargetPose(new Pose(48, 77, Math.toRadians(130)), new Pose(1.5, 5, 3));
                        state++;
                        timer.reset();
                    }
                    break;
                case 5:
                    if (timer.milliseconds() >= 900) {
                        robot.wrist.setState(Wrist.States.Collect);
                        state++;
                    }
                    break;
                case 6:
                    if (robot.drive.isDone()) {
                        robot.drive.setTargetPose(new Pose(48, 35, Math.toRadians(70)), new Pose(6, 6, 5));
                        state++;
                    }
                    break;
                case 7:
                    if (robot.drive.isDone()) {
                        robot.wrist.setState(Wrist.States.Wait);
                        robot.drive.setTargetPose(new Pose(73.5, 77, Math.toRadians(130)), new Pose(1.5, 5, 3));
                        state++;
                        timer.reset();
                    }
                    break;
                case 8:
                    if (timer.milliseconds() >= 901) {
                        robot.wrist.setState(Wrist.States.Collect);
                        state++;
                    }
                    break;
                case 9:
                    if (robot.drive.isDone()) {
                        robot.drive.setTargetPose(new Pose(73.5, 28, Math.toRadians(70)), new Pose(5, 5, 5));
                        state++;
                    }
                    break;
                case 10:
                    if (robot.drive.isDone()) {
                        robot.slides.setState(LinearSlides.States.Retracted);
                        robot.wrist.setState(Wrist.States.Wait);
                        robot.drive.setTargetPose(new Pose(50, 2, Math.toRadians(180)), new Pose(3, 1, 3));
                        headignMultiplier = 6;
                        state++;
                    }
                    break;
                case 11:
                    if (robot.drive.isDone()) {
                        robot.setAction(Robot.Actions.ScoreSpecimenHigh);
                        state++;
                        timer.reset();
                    }
                    break;
                case 12:
                    if (timer.milliseconds() >= 300) {
                        lateralMultiplier = 3;
                        headignMultiplier = 4;
                        robot.drive.setMaxPower(1);
                        robot.drive.setTargetPose(new Pose(-29 + 4.5 * i, 77, 0), new Pose(2, 1, 3));
                        state++;
                    }
                    break;
                case 13:
                    if (robot.drive.isDone()) {
                        robot.setAction(Robot.Actions.ScoreSpecimenHigh);
                        if (i == 3)
                            state = 15;
                        else
                            state++;
                        timer.reset();
                    }
                    break;
                case 14:
                    if (robot.isClawOpened() && timer.milliseconds() >= 100) {
                        robot.setAction(Robot.Actions.Collect);
                        headignMultiplier = 4.5;
                        lateralMultiplier = 3.5;
                        robot.drive.setTargetPose(new Pose(50, 2, Math.toRadians(180)), new Pose(3, 1, 3));
                        robot.drive.setMaxPower(1);
                        i++;
                        state = 11;
                    }
                    break;
                case 15:
                    if (robot.isClawOpened()) {
                        lateralMultiplier = 3;
                        robot.drive.setTargetPose(new Pose(50, 40, 0));
                        robot.setAction(Robot.Actions.Collect);
                        robot.slides.setState(LinearSlides.States.Extended);
                        robot.wrist.setState(Wrist.States.Park);
                        state++;
                    }
                    break;
            }
            if (robot.intakeSample.getPower() == Constants.IntakeActivePower.intakeScore)
                robot.intakeSample.setState(IntakeActive.States.Wait);
            robot.update();
        }
    }
}
