package org.firstinspires.ftc.teamcode.OpModes.SpeediTuningAndTesting.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;

@TeleOp
@Config
public class MaxForwardVelocity extends LinearOpMode {
    MecanumDrive drive;
    public  static double accelerationTime = 1300;//MS
    double maxVelocity = -1;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new MecanumDrive(hardwareMap, new Pose(), telemetry, true, true);
        telemetry.addData("Velocity", drive.localizer.getVelocity().getY());
        telemetry.update();
        drive.setRunMode(MecanumDrive.RunMode.MANUAL);
        waitForStart();

        drive.motors.setMotorPower(1, 1, 1, 1);


        long startTime = System.currentTimeMillis();
        while (opModeIsActive()) {
            drive.update();
            telemetry.addData("Velocity", drive.localizer.getVelocity().getY());
            telemetry.update();
            maxVelocity = Math.max(drive.localizer.getVelocity().getY(), maxVelocity);
            if (System.currentTimeMillis() - startTime > accelerationTime) {
                drive.motors.setMotorPower(0, 0, 0, 0);
                break;
            }
        }
        while (opModeIsActive()) {
            telemetry.addData("Max Velocity", maxVelocity);
            telemetry.update();
        }
    }
}
