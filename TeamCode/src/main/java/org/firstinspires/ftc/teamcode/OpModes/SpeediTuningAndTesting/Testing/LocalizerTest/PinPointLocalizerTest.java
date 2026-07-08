package org.firstinspires.ftc.teamcode.OpModes.SpeediTuningAndTesting.Testing.LocalizerTest;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Follower.SpeediDrive;
import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Localizer.PinPointLocalizer;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;

@TeleOp
public class PinPointLocalizerTest extends LinearOpMode {
    SpeediDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //the drive's own localizer is used directly: a second PinPointLocalizer on the same device
        //would reset it again and double the I2C traffic every loop
        drive = new SpeediDrive(hardwareMap, new Pose(), telemetry, true);
        drive.setRunMode(SpeediDrive.RunMode.MANUAL);
        PinPointLocalizer localizer = drive.localizer;

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                drive.resetPosition(new Pose());
            }
            drive.driveFieldCentric(gamepad1, localizer.getPoseEstimate().getHeading(AngleUnit.RADIANS));

            telemetry.addData("Predicted pos", localizer.getPredictedPoseEstimate());
            telemetry.addData("Pos actuala", localizer.getPoseEstimate());
            telemetry.addData("velocity prelucrata", localizer.getVelocity().toString());
            telemetry.addData("velocity raw", localizer.getRawVelocity().toString());

            telemetry.addData("If this isn't between 900 and 2000 the pinpoint is broken", localizer.odo.getFrequency());
            telemetry.update();

            drive.update();
        }
    }
}
