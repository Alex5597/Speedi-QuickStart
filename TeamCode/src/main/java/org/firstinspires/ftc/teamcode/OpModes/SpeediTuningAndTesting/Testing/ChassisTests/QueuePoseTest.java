package org.firstinspires.ftc.teamcode.OpModes.SpeediTuningAndTesting.Testing.ChassisTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Follower.SpeediDrive;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;

import java.util.LinkedList;
import java.util.Queue;

@TeleOp
public class QueuePoseTest extends LinearOpMode {
    SpeediDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SpeediDrive(hardwareMap, new Pose(), telemetry, true);
        Queue<Pose> targetPoses = new LinkedList<>();
        targetPoses.add(new Pose(100, 0, DistanceUnit.CM));
        targetPoses.add(new Pose(100, 130, DistanceUnit.CM));
        waitForStart();
        drive.setTargetsList(targetPoses, 70);
        while (opModeIsActive()) {
            drive.currentPosTelemetry(true);
            drive.targetTelemetry(false);
            drive.update();
        }
    }
}
