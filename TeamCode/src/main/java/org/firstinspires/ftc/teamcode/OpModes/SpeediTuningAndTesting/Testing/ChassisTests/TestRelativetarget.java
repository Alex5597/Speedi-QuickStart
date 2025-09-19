package org.firstinspires.ftc.teamcode.OpModes.SpeediTuningAndTesting.Testing.ChassisTests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Follower.SpeediDrive;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;


@Disabled
@TeleOp
public class TestRelativetarget extends LinearOpMode {
    SpeediDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SpeediDrive(hardwareMap, new Pose(0, 0, DistanceUnit.CM, 45, AngleUnit.DEGREES), telemetry, true);

        waitForStart();
        drive.driveRelativelyToRobotPos(50, 30, 5, true);
        while (opModeIsActive()) {
            drive.update();
            drive.currentPosTelemetry(false);
        }
    }
}
