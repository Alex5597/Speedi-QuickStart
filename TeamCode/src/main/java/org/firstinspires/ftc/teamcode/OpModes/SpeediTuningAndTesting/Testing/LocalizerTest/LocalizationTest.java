package org.firstinspires.ftc.teamcode.OpModes.SpeediTuningAndTesting.Testing.LocalizerTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Follower.GoToPoint;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;

@TeleOp
public class LocalizationTest extends LinearOpMode {
    GoToPoint drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new GoToPoint(hardwareMap, new Pose(), telemetry, true);

        try {
            sleep(500);
        } catch (RuntimeException e) {
            throw new RuntimeException(e);
        }
        drive.setRunMode(GoToPoint.RunMode.MANUAL);
        waitForStart();

        while (opModeIsActive()) {
            drive.motors.drive(gamepad1);
            drive.currentPosTelemetry(false);
            drive.update();
        }
    }
}
