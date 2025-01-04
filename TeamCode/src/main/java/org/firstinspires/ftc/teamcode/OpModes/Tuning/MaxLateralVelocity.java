package org.firstinspires.ftc.teamcode.OpModes.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.core.Modules.OutTake_Intake.IntakeActive;
import org.firstinspires.ftc.teamcode.core.Modules.OutTake_Intake.LinearSlides;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;

@TeleOp
public class MaxLateralVelocity extends LinearOpMode {
    MecanumDrive drive;
    LinearSlides slides;
    IntakeActive intakeActive;
    double accelerationTime = 2000; //MS
    double maxVelocity = -1;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        slides = new LinearSlides(hardwareMap, true);
        drive = new MecanumDrive(hardwareMap, new Pose(), telemetry, true, true);
        telemetry.addData("Velocity", drive.localizer.getVelocity().getY());
        telemetry.update();
        waitForStart();
        drive.motors.setMotorPower(1, -1, -1, 1);

        long startTime = System.currentTimeMillis();
        while (opModeIsActive()) {
            drive.update();
            telemetry.addData("Velocity", drive.localizer.getVelocity().getX());
            telemetry.update();
            maxVelocity = Math.max(drive.localizer.getVelocity().getX(), maxVelocity);
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
