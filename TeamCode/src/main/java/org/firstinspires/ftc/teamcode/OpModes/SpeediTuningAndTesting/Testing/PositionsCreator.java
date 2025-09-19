package org.firstinspires.ftc.teamcode.OpModes.SpeediTuningAndTesting.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Follower.SpeediDrive;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;

import java.util.ArrayList;

@TeleOp
@Config
public class PositionsCreator extends LinearOpMode {
    ElapsedTime timer1 = new ElapsedTime(), timer2 = new ElapsedTime();
    SpeediDrive drive;
    ArrayList<Pose> positions = new ArrayList<>();
    double xSTartPoseInCm = 0, yStartPoseInCm = 0, headingStartPoseInDegrees = 0;
    Pose startPose = new Pose(xSTartPoseInCm, yStartPoseInCm, DistanceUnit.CM, headingStartPoseInDegrees, AngleUnit.DEGREES);

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SpeediDrive(hardwareMap, startPose, telemetry, true, true);
        drive.setRunMode(SpeediDrive.RunMode.MANUAL);
        while (!isStarted()) {
            telemetry.addLine("Set the start position by using the visualizer and place your robot on the exact position you want to start the auto and copy the coordinates");
            telemetry.update();
        }
        waitForStart();
        telemetry.clearAll();
        telemetry.update();

        while (opModeIsActive()) {
            drive.drive(gamepad1);
            drive.update();
            if ((gamepad1.a || gamepad1.cross) && timer1.milliseconds() >= 300) {
                startPose = new Pose(xSTartPoseInCm, yStartPoseInCm, DistanceUnit.CM, headingStartPoseInDegrees, AngleUnit.DEGREES);
                drive.resetPosition(startPose);
                timer1.reset();
            }
            if ((gamepad1.b || gamepad1.circle) && timer2.milliseconds() >= 300) {
                positions.add(drive.getCurrentPos());
                timer2.reset();
                telemetry.clearAll();
                telemetry.update();
                telemetry.addLine("Position saved, wait for the other telemetry to appear and you can continue/stop");
                telemetry.update();
                wait(1000);
                telemetry.clearAll();
                telemetry.update();
            }
            if (gamepad1.dpad_up)
                break;
            telemetry.addLine("If you are done saving positions press dpad_up");
            telemetry.addLine("For reseting the starting position press A on xbox controller or X on PS controller");
            telemetry.addLine("For saving a position press B on xbox controllers or ⭕ on PS controllers");
            telemetry.update();
        }
        telemetry.clearAll();
        telemetry.update();
        while (opModeIsActive()) {
            for (int i = 0; i < positions.size(); i++) {
                telemetry.addLine("Pose pose" + (i + 1) + "= new Pose(" + positions.get(i).getX(DistanceUnit.CM) + ", " + positions.get(i).getY(DistanceUnit.CM) + ", DistanceUnit.CM, " + positions.get(i).getHeading(AngleUnit.DEGREES) + ", AngleUnit.DEGREES)\n");
            }
            telemetry.update();
        }
    }
}
