package org.firstinspires.ftc.teamcode.core;

import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.IntakePassivePositions.intakePassiveClosePose;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.IntakePassivePositions.intakePassiveOpenPose;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OpModes.TeleOps.TeleOpBun;
import org.firstinspires.ftc.teamcode.core.Modules.Climb.ClimbModule;
import org.firstinspires.ftc.teamcode.core.Modules.OutTake_Intake.Arm;
import org.firstinspires.ftc.teamcode.core.Modules.OutTake_Intake.Bucket;
import org.firstinspires.ftc.teamcode.core.Modules.OutTake_Intake.IntakePassive;
import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.core.Modules.OutTake_Intake.IntakeActive;
import org.firstinspires.ftc.teamcode.core.Modules.OutTake_Intake.Lift;
import org.firstinspires.ftc.teamcode.core.Modules.OutTake_Intake.LinearSlides;
import org.firstinspires.ftc.teamcode.core.Modules.OutTake_Intake.Wrist;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;

import java.util.List;

public class Robot {
    ElapsedTime timerForOpeningClaw = new ElapsedTime(), timerForSlides = new ElapsedTime(), timerForWrist = new ElapsedTime(), timerLift = new ElapsedTime();
    public MecanumDrive drive;
    public Arm arm;
    public IntakeActive intakeSample;
    public IntakePassive intakeSpecimen;
    public Lift lift;
    public ClimbModule climb;
    public Wrist wrist;
    public LinearSlides slides;
    public Bucket bucket;
    List<LynxModule> allHubs;
    boolean isAutonomous;

    public enum Actions {
        Collect,
        ScoreSpecimenHigh,
        ScoreSpecimenLow,
        ScoreSampleHigh,
        ScoreSampleLow,
        SlidesExtended,
        SlidesRetracted,
        InstantPutWristDown,
        InstantRetract,
        GOUP
    }

    boolean shouldOpenClaw = false;
    boolean shouldMoveWristForCollect = false;
    boolean shouldMoveWristForTransfer = false;
    boolean shouldExtendLinkage = false;
    boolean shouldExtend = false;
    boolean shouldRaiseLiftHigh = false, shouldRaiseLiftLow = false;

    public Robot(HardwareMap hardwareMap, Pose startPose, Telemetry telemetry, boolean isAutonomous, IntakeActive.Color allianceColor) {
        this.isAutonomous = isAutonomous;
        if (isAutonomous) {
            intakeSpecimen = new IntakePassive(hardwareMap, intakePassiveClosePose);
            drive = new MecanumDrive(hardwareMap, startPose, telemetry, true, true);
        } else {
            intakeSpecimen = new IntakePassive(hardwareMap, intakePassiveOpenPose);
            drive = new MecanumDrive(hardwareMap, startPose, telemetry, true, false);
        }
        climb = new ClimbModule(hardwareMap, telemetry);
        lift = new Lift(hardwareMap);
        lift.resetEnc();
        arm = new Arm(hardwareMap);
        intakeSample = new IntakeActive(hardwareMap, allianceColor);
        wrist = new Wrist(hardwareMap, isAutonomous);
        slides = new LinearSlides(hardwareMap, isAutonomous);
        bucket = new Bucket(hardwareMap, intakeSample);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public void update() {
        if (shouldOpenClaw && timerForOpeningClaw.milliseconds() >= 350) {
            intakeSpecimen.open();
            shouldOpenClaw = false;
        }
        if (shouldMoveWristForCollect && timerForWrist.milliseconds() >= 400) {
            wrist.setState(Wrist.States.Collect);
            shouldMoveWristForCollect = false;
        }
        if (shouldMoveWristForTransfer && timerForWrist.milliseconds() >= 300) {
            slides.setState(LinearSlides.States.Retracted);
            intakeSample.setState(IntakeActive.States.Collect);
            bucket.setState(Bucket.States.Wait);
            shouldMoveWristForTransfer = false;
        }
        if (shouldExtend && timerForSlides.milliseconds() >= 400) {
            slides.setState(LinearSlides.States.Extended);
            shouldExtend = false;
        }
        if (shouldRaiseLiftHigh && timerLift.milliseconds() >= 300) {
            lift.setState(Lift.States.HighChamber);
            shouldRaiseLiftHigh = false;
        }
        if (shouldRaiseLiftLow && timerLift.milliseconds() >= 300) {
            lift.setState(Lift.States.LowChamber);
            shouldRaiseLiftLow = false;
        }
        drive.update();
        wrist.update();
        arm.update();
        intakeSpecimen.update();
        lift.update();
        intakeSample.update();
        slides.update();
        bucket.update();
        climb.update();

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }//TODO A NU SE SCHIMBA
    }

    public boolean isClawOpened() {
        return !shouldOpenClaw;
    }

    public void setAction(Actions action) {
        switch (action) {
            case Collect:
                lift.setState(Lift.States.Collect);
                arm.setState(Arm.States.Collect);
                bucket.setCanScore(false);
                bucket.setState(Bucket.States.Wait);
                wrist.setState(Wrist.States.Wait);
                TeleOpBun.state = TeleOpBun.States.Specimens;
                shouldExtendLinkage = true;
                break;
            case ScoreSampleHigh:
                slides.setState(LinearSlides.States.Aux);
                if (lift.getState() != Lift.States.HighBasket) {
                    bucket.setCanScore(true);
                    wrist.setState(Wrist.States.Wait);
                    lift.setState(Lift.States.HighBasket);
                    arm.setState(Arm.States.Place);
                    bucket.setState(Bucket.States.Hold);
                } else {
                    wrist.setState(Wrist.States.Wait);
                    lift.setState(Lift.States.HighBasket);
                    arm.setState(Arm.States.Place);
                    bucket.setState(Bucket.States.Wait);
                    TeleOpBun.state = TeleOpBun.States.Specimens;
                }
                break;
            case ScoreSampleLow:
                slides.setState(LinearSlides.States.Aux);
                if (lift.getState() != Lift.States.LowBasket) {
                    bucket.setCanScore(true);
                    wrist.setState(Wrist.States.Wait);
                    lift.setState(Lift.States.LowBasket);
                    arm.setState(Arm.States.Place);
                    bucket.setState(Bucket.States.Hold);
                } else {
                    wrist.setState(Wrist.States.Wait);
                    lift.setState(Lift.States.LowBasket);
                    arm.setState(Arm.States.Place);
                    bucket.setState(Bucket.States.Wait);
                    TeleOpBun.state = TeleOpBun.States.Specimens;
                }
                break;
            case ScoreSpecimenHigh:
                if (lift.getState() == Lift.States.Collect && !shouldRaiseLiftHigh) {
                    intakeSpecimen.close();
                    arm.setState(Arm.States.Collect);
                    wrist.setState(Wrist.States.Wait);
                    shouldRaiseLiftHigh = true;
                    timerLift.reset();
                } else {
                    shouldOpenClaw = true;
                    lift.setState(Lift.States.ScoreHighSpecimen);
                    arm.setState(Arm.States.Collect);
                    wrist.setState(Wrist.States.Wait);
                    timerForOpeningClaw.reset();
                }
                break;
            case ScoreSpecimenLow:
                if (lift.getState() == Lift.States.Collect && !shouldRaiseLiftLow) {
                    intakeSpecimen.close();
                    arm.setState(Arm.States.Collect);
                    wrist.setState(Wrist.States.Wait);
                    shouldRaiseLiftLow = true;
                    timerLift.reset();
                } else {
                    shouldOpenClaw = true;
                    lift.setState(Lift.States.Collect);
                    arm.setState(Arm.States.Collect);
                    wrist.setState(Wrist.States.Wait);
                    timerForOpeningClaw.reset();
                }
                break;
            case SlidesExtended:
                wrist.setState(Wrist.States.Wait);
                slides.setState(LinearSlides.States.Extended);
                intakeSample.setState(IntakeActive.States.Collect);
                bucket.setState(Bucket.States.Wait);
                shouldMoveWristForCollect = true;
                timerForWrist.reset();
                break;
            case SlidesRetracted:
                if (intakeSample.getColor() == IntakeActive.Color.Yellow)
                    wrist.setState(Wrist.States.Transfer);
                else
                    wrist.setState(Wrist.States.Wait);
                shouldMoveWristForTransfer = true;
                timerForWrist.reset();
                break;
            case InstantPutWristDown:
                wrist.setState(Wrist.States.Collect);
                intakeSample.setState(IntakeActive.States.Collect);
                shouldExtend = true;
                timerForSlides.reset();
                break;
            case InstantRetract:
                slides.setState(LinearSlides.States.Retracted);
                wrist.setState(Wrist.States.Wait);
                break;
            case GOUP:
                lift.setState(Lift.States.GOUP);
                break;
        }
    }
}
