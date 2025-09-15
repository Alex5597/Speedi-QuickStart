package org.firstinspires.ftc.teamcode.OpModes.SpeediTuningAndTesting.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Follower.GoToPoint;

@TeleOp
public class SWITCH_FROM_STATIC_TO_KINETIC_FRICTIONTuner extends LinearOpMode {
    GoToPoint drive;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new GoToPoint(hardwareMap, telemetry, true);
        drive.setRunMode(GoToPoint.RunMode.MANUAL);
        waitForStart();

        drive.motors.goAtMinimumPower();
        long startTime = System.currentTimeMillis(), stopTime;
        while (true) {
            if (drive.localizer.getVelocity().getMagnitude() >= 15) {
                stopTime = System.currentTimeMillis();
                break;
            }
            drive.localizer.update();
        }
        drive.motors.setMotorPower(0, 0, 0, 0);
        while (opModeIsActive()) {
            telemetry.addData("Time to start", stopTime - startTime);
            telemetry.update();
        }
    }
}
