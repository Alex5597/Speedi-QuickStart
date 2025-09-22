package org.firstinspires.ftc.teamcode.OpModes.SpeediTuningAndTesting.Testing.LocalizerTest;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Follower.SpeediDrive;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;

@TeleOp
public class LocalizationTest extends LinearOpMode {
    SpeediDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SpeediDrive(hardwareMap, new Pose(), telemetry, true);

        try {
            sleep(500);
        } catch (RuntimeException e) {
            throw new RuntimeException(e);
        }
        drive.setRunMode(SpeediDrive.RunMode.MANUAL);
        waitForStart();

        while (opModeIsActive()) {
            drive.drive(gamepad1);
            drive.currentPosTelemetry(false);
            drive.update();
        }
    }
}
