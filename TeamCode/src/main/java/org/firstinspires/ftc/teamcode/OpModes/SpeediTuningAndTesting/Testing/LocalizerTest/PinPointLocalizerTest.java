package org.firstinspires.ftc.teamcode.OpModes.SpeediTuningAndTesting.Testing.LocalizerTest;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Follower.SpeediDrive;
import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Localizer.PinPointLocalizer;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;
import org.firstinspires.ftc.teamcode.core.Util.utils.DrawRobot;

@TeleOp
public class PinPointLocalizerTest extends LinearOpMode {
    PinPointLocalizer localizer;
    SpeediDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        localizer = new PinPointLocalizer(hardwareMap, new Pose());
        drive = new SpeediDrive(hardwareMap, new Pose(), telemetry, false);

        waitForStart();

        while (opModeIsActive()) {
            DrawRobot.drawRobot(localizer.getPoseEstimate());
            if (gamepad1.a) {
                localizer.resetPosition(new Pose());
                drive.resetPosition(new Pose());
            }
            drive.motors.driveFieldCentric(gamepad1, 0);

            telemetry.addData("Predicted pos", localizer.getPredictedPoseEstimate());
            telemetry.addData("Pos actuala", localizer.getPoseEstimate());
            telemetry.addData("velocity prelucrata",localizer.getVelocity().toString());
            telemetry.addData("velocity raw",localizer.getRawVelocity().toString());

            telemetry.addData("If this isn't between 900 and 2000 the pinpoint is broken", localizer.odo.getFrequency());
            telemetry.update();


            localizer.update();
            drive.update();
        }
    }
}
