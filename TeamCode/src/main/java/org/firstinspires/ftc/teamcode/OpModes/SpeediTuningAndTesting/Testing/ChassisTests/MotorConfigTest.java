package org.firstinspires.ftc.teamcode.OpModes.SpeediTuningAndTesting.Testing.ChassisTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Follower.SpeediDrive;

@TeleOp
public class MotorConfigTest extends LinearOpMode {
    SpeediDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SpeediDrive(hardwareMap, telemetry,true);
        drive.setRunMode(SpeediDrive.RunMode.MANUAL);
        waitForStart();

        while (opModeIsActive()) {
            for (int i = 0; i < 4 || isStopRequested(); i++) {
                telemetry.addLine(drive.motors.motors.get(i).name + " should be moving forward");
                telemetry.update();
                drive.motors.motors.get(i).setPowerForced(1);
                sleep(5000);
                drive.motors.motors.get(i).setPowerForced(0);
            }
        }
    }
}
