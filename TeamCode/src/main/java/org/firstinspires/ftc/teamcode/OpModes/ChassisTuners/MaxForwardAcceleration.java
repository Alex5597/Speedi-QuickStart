package org.firstinspires.ftc.teamcode.OpModes.ChassisTuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;

@Config
@TeleOp
public class MaxForwardAcceleration extends LinearOpMode {
    MecanumDrive drive;
    double maxAccel = 0;
    public static double maxAllowedTimeInSecs = 1.1;
    ElapsedTime timer = new ElapsedTime();
    double lastTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new MecanumDrive(hardwareMap, new Pose(), telemetry, true, false);

        waitForStart();
        drive.motors.setMotorPower(1, 1, 1, 1);
        timer.reset();
        while (timer.seconds() <= maxAllowedTimeInSecs) {
            double currAccel = drive.localizer.getVelocity().getMagnitude() / (timer.seconds() - lastTime);
            if (maxAccel < currAccel)
                maxAccel = currAccel;
            lastTime = timer.seconds();
            drive.update();
        }
        drive.motors.setMotorPower(0, 0, 0, 0);
        drive.update();
        while (opModeIsActive()) {
            telemetry.addData("Max acceleration", maxAccel);
            telemetry.update();
        }
    }
}
