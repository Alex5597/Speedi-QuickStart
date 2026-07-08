package org.firstinspires.ftc.teamcode.OpModes.SpeediTuningAndTesting.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Follower.SpeediDrive;

@Config
@TeleOp
public class SWITCH_FROM_STATIC_TO_KINETIC_FRICTIONTuner extends LinearOpMode {
    public static double movingVelocityThreshold = 15;//cm/s at which the robot counts as moving

    SpeediDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SpeediDrive(hardwareMap, telemetry, true);
        drive.setRunMode(SpeediDrive.RunMode.MANUAL);
        waitForStart();

        drive.motors.goAtMinimumPower();
        long startTime = System.currentTimeMillis(), stopTime = startTime;
        while (opModeIsActive()) {
            if (drive.localizer.getVelocity().getMagnitude() >= movingVelocityThreshold) {
                stopTime = System.currentTimeMillis();
                break;
            }
            drive.localizer.update();
        }
        drive.motors.setMotorPower(0, 0, 0, 0);
        while (opModeIsActive()) {
            telemetry.addData("Time to start (copy into SWITCH_FROM_STATIC_TO_KINETIC_FRICTION)", stopTime - startTime);
            telemetry.update();
        }
    }
}
