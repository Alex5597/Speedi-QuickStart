package org.firstinspires.ftc.teamcode.OpModes.Testing.LocalizerTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;

@TeleOp
public class LocalizationTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose(0, 0, 0), telemetry,true);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a)
                drive.resetPosition(new Pose());
            drive.motors.drive(gamepad1);

            drive.update();
            telemetry.addData("Velo x",drive.localizer.getVelocity().getX());
            telemetry.addData("Velo y",drive.localizer.getVelocity().getY());
            telemetry.addData("Power Forward", gamepad1.left_stick_y);
            telemetry.addData("Power Lateral", gamepad1.left_stick_x);
            telemetry.addData("Power Angular", gamepad1.right_stick_x);
            telemetry.update();
        }
    }
}
