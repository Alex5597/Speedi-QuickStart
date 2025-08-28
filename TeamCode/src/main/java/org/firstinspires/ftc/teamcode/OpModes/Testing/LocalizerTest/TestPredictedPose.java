package org.firstinspires.ftc.teamcode.OpModes.Testing.LocalizerTest;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Drive.Chassis;
import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Localizer.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;
import org.firstinspires.ftc.teamcode.core.Util.utils.DrawRobot;

@Disabled
@TeleOp
public class TestPredictedPose extends LinearOpMode {
    TwoWheelLocalizer localizer;
    Chassis chassis;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        localizer = new TwoWheelLocalizer(hardwareMap, new Pose(), telemetry);
        chassis = new Chassis(hardwareMap, false);
        waitForStart();

        while (opModeIsActive()) {
            chassis.drive(gamepad1);
            chassis.update();
            localizer.update();
            telemetry.addData("Pose", localizer.getPoseEstimate().toString());
            telemetry.addData("Predicted pose", localizer.getPredictedPoseEstimate().toString());
            telemetry.addData("x Velocity", localizer.getVelocity().getX());
            telemetry.addData("y Velocity", localizer.getVelocity().getY());
            telemetry.update();
        }
    }
}
