package org.firstinspires.ftc.teamcode.OpModes.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;

@TeleOp
public class MaxLateralVelocity extends LinearOpMode {
    MecanumDrive drive;
    double accelerationTime = 1500; //MS
    double maxVelocity = -1;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new MecanumDrive(hardwareMap, new Pose(), telemetry, true, true);

        waitForStart();
        drive.motors.setMotorPower(1, -1, -1, 1);

        long startTime = System.currentTimeMillis();
        while (opModeIsActive()) {
            maxVelocity = Math.max(drive.getLocalizerInstance().getVelocity().getX(), maxVelocity);
            if (System.currentTimeMillis() - startTime > accelerationTime) {
                drive.motors.setMotorPower(0, 0, 0, 0);
                break;
            }
            drive.localizer.update();
        }

        while (opModeIsActive()) {
            telemetry.addData("Max Velocity", maxVelocity);
            telemetry.update();
        }
    }
}
