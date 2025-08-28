package org.firstinspires.ftc.teamcode.OpModes.SpeediTuningAndTesting.Testing.LocalizerTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;

@TeleOp
public class LocalizationTest extends LinearOpMode {
    MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, new Pose(0, 0, Math.toRadians(-35)), telemetry, true);

        try {
            sleep(500);
        } catch (RuntimeException e) {
            throw new RuntimeException(e);
        }
        drive.setRunMode(MecanumDrive.RunMode.MANUAL);
        waitForStart();

        while (opModeIsActive()) {
            drive.motors.drive(gamepad1);
            drive.currentPosTelemetry(false);
            drive.update();
        }
    }
}
