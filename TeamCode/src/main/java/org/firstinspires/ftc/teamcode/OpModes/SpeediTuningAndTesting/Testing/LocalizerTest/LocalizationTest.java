package org.firstinspires.ftc.teamcode.OpModes.SpeediTuningAndTesting.Testing.LocalizerTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Follower.SpeediDrive;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;

@TeleOp
public class LocalizationTest extends LinearOpMode {
    SpeediDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SpeediDrive(hardwareMap, new Pose(), telemetry, true);

        try {
            sleep(500);
        } catch (RuntimeException e) {
            throw new RuntimeException(e);
        }
        drive.setRunMode(SpeediDrive.RunMode.MANUAL);
        waitForStart();

        while (opModeIsActive()) {
            drive.motors.drive(gamepad1);
            drive.currentPosTelemetry(false);
            drive.update();
        }
    }
}
