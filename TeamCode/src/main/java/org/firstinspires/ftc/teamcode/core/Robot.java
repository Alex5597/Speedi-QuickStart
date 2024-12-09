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
    ElapsedTime timerForOpeningClaw = new ElapsedTime(), timerForSlides = new ElapsedTime(), timerForWrist = new ElapsedTime();
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
        SlidesRetracted
    }

    boolean shouldOpenClaw = false;
    boolean shouldMoveWristForCollect = false;
    boolean shouldMoveWristForTransfer = false;
    boolean shouldExtendLinkage = false;

    public Robot(HardwareMap hardwareMap, Pose startPose, Telemetry telemetry, boolean isAutonomous, IntakeActive.Color allianceColor) {
        this.isAutonomous = isAutonomous;
        if (isAutonomous) {
            intakeSpecimen = new IntakePassive(hardwareMap, intakePassiveClosePose);
            drive = new MecanumDrive(hardwareMap, startPose, telemetry, false, true);
        } else {
            intakeSpecimen = new IntakePassive(hardwareMap, intakePassiveOpenPose);
            drive = new MecanumDrive(hardwareMap, startPose, telemetry, true, false);
        }
        climb = new ClimbModule(hardwareMap);
        lift = new Lift(hardwareMap);
        lift.resetEnc();
        arm = new Arm(hardwareMap);
        intakeSample = new IntakeActive(hardwareMap, allianceColor);
        wrist = new Wrist(hardwareMap);
        slides = new LinearSlides(hardwareMap);
        bucket = new Bucket(hardwareMap, intakeSample);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public void update() {
        if (shouldOpenClaw && timerForOpeningClaw.milliseconds() >= 400) {
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
        drive.update();
        wrist.update();
        arm.update();
        intakeSpecimen.update();
        lift.update();
        intakeSample.update();
        slides.update();
        bucket.update();

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
                bucket.setState(Bucket.States.Wait);
                wrist.setState(Wrist.States.Wait);
                TeleOpBun.state = TeleOpBun.States.Specimens;
                shouldExtendLinkage = true;
                break;
            case ScoreSampleHigh:
                slides.setState(LinearSlides.States.Aux);
                if (lift.getState() != Lift.States.HighChamber) {
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
                if (shouldExtendLinkage) {
                    slides.setState(LinearSlides.States.Aux);
                    shouldExtendLinkage = false;
                    timerForSlides.reset();
                } else if (timerForSlides.milliseconds() >= 500)
                    if (lift.getState() != Lift.States.HighChamber) {
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
                if (shouldExtendLinkage) {
                    slides.setState(LinearSlides.States.Aux);
                    shouldExtendLinkage = false;
                    timerForSlides.reset();
                } else if (timerForSlides.milliseconds() >= 500)
                    if (lift.getState() != Lift.States.HighChamber) {
                        lift.setState(Lift.States.HighChamber);
                        arm.setState(Arm.States.Collect);
                        wrist.setState(Wrist.States.Wait);
                    } else {
                        lift.setState(Lift.States.ScoreHighSpecimen);
                        arm.setState(Arm.States.Collect);
                        wrist.setState(Wrist.States.Wait);
                        shouldOpenClaw = true;
                        timerForOpeningClaw.reset();
                    }
                break;
            case ScoreSpecimenLow:
                if (lift.getState() != Lift.States.LowChamber) {
                    lift.setState(Lift.States.LowChamber);
                    arm.setState(Arm.States.Collect);
                    wrist.setState(Wrist.States.Wait);
                } else {
                    lift.setState(Lift.States.Collect);
                    arm.setState(Arm.States.Collect);
                    wrist.setState(Wrist.States.Wait);
                    shouldOpenClaw = true;
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
                wrist.setState(Wrist.States.Wait);
                shouldMoveWristForTransfer = true;
                timerForWrist.reset();
                break;
        }
    }
}
