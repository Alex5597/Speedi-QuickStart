package org.firstinspires.ftc.teamcode.OpModes.SpeediTuningAndTesting.Testing.ChassisTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Follower.SpeediDrive;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;

@TeleOp
public class TestNoGoZone extends LinearOpMode {
    SpeediDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SpeediDrive(hardwareMap, new Pose(-120, -36.5, DistanceUnit.CM), telemetry, true);
        //the zone must leave a corridor inside the field (corners inflated by half the robot size have to stay reachable),
        //otherwise the avoidance has no corner to route through and throws "no go zone can not be avoided"
        drive.setNoGoZone(new Pose(-183, 90.5, DistanceUnit.CM), new Pose(60, 31.5, DistanceUnit.CM), 70);
        telemetry.addLine("gata");
        telemetry.update();
        waitForStart();
        drive.setTargetPose(new Pose(-120, 120, DistanceUnit.CM, 0, AngleUnit.DEGREES), new Pose(3, 3, DistanceUnit.CM, 3, AngleUnit.DEGREES), true);
        while (opModeIsActive()) {
            drive.update();
            drive.noGoZoneTelemetry(true);
            drive.targetTelemetry(true);
            drive.currentPosTelemetry(false);
        }
    }
}
