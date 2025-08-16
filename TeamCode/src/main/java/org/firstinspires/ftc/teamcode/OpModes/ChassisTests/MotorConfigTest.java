package org.firstinspires.ftc.teamcode.OpModes.ChassisTests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Drive.MecanumDrive;

@TeleOp
public class MotorConfigTest extends LinearOpMode {
    MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, telemetry,true);

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
