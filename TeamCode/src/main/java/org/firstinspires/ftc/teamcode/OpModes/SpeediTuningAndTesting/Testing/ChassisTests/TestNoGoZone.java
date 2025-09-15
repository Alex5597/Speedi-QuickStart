package org.firstinspires.ftc.teamcode.OpModes.SpeediTuningAndTesting.Testing.ChassisTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Follower.GoToPoint;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;

@TeleOp
public class TestNoGoZone extends LinearOpMode {
    GoToPoint drive;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new GoToPoint(hardwareMap, new Pose(-146, -162, DistanceUnit.CM), telemetry, true);
        drive.setNoGoZone(new Pose(-230, 56, DistanceUnit.CM), new Pose(-30, -105, DistanceUnit.CM), 0);
        telemetry.addLine("gata");
        telemetry.update();
        waitForStart();
        drive.setTargetPose(new Pose(-120, -110, DistanceUnit.CM, 0, AngleUnit.DEGREES), new Pose(3, 3, DistanceUnit.CM, 3, AngleUnit.DEGREES), true);
        while (opModeIsActive()) {
            drive.update();
            drive.noGoZoneTelemetry(true);
            drive.targetTelemetry(true);
            drive.currentPosTelemetry(false);
        }
    }
}
