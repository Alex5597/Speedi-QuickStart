package org.firstinspires.ftc.teamcode.OpModes.SpeediTuningAndTesting.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Follower.SpeediDrive;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;

@Config
@TeleOp
public class ChassisPIDTuner extends LinearOpMode {
    SpeediDrive drive;
    enum State {
        DRIVING,
        AUTO
    }

    State state;
    Pose startPose = new Pose(0, 0, DistanceUnit.CM, Math.toRadians(0), AngleUnit.RADIANS);

    public static double xTargetPos = 101, yTargetPos = 15, angleTargetPos = 0;//TODO Change how you want
    Pose targetPos = new Pose(xTargetPos, yTargetPos,DistanceUnit.CM, angleTargetPos, AngleUnit.DEGREES);
    int traj = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SpeediDrive(hardwareMap, startPose, telemetry, true);
        state = State.AUTO;

        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            switch (state) {
                case DRIVING:
                    if (gamepad1.b) {
                        targetPos = new Pose(xTargetPos, yTargetPos,DistanceUnit.CM, angleTargetPos, AngleUnit.DEGREES);
                        drive.resetPosition(startPose);
                        state = State.AUTO;
                        traj = 0;
                    }
                    drive.motors.drive(gamepad1);
                    drive.update();
                    drive.currentPosTelemetry(true);
                    drive.PinPointErrorTelemetry(false);
                    break;
                case AUTO:
                    targetPos = new Pose(xTargetPos, yTargetPos,DistanceUnit.CM, angleTargetPos, AngleUnit.DEGREES);
                    drive.setTargetPose(targetPos,true);
                    traj = 0;
                    while (opModeIsActive()) {
                        switch (traj) {
                            case 0:
                                if (drive.isDone()) {
                                    traj++;
                                    drive.setTargetPose(startPose,true);
                                }
                                break;
                        }
                        if (drive.isDone() && traj == 1)
                            break;
                        drive.update();
                        if (gamepad1.a) {
                            state = State.DRIVING;
                            drive.setRunMode(SpeediDrive.RunMode.MANUAL);
                            drive.stop();
                            break;
                        }
                        telemetry.addData("Velocity", drive.localizer.getVelocity().toString());
                        telemetry.addData("Predicted pose", drive.localizer.getPredictedPoseEstimate());
                        drive.PinPointErrorTelemetry(true);
                        drive.currentPosTelemetry(true);
                        drive.errorTelemetry(false);
                    }
                    break;
            }
        }
    }
}
