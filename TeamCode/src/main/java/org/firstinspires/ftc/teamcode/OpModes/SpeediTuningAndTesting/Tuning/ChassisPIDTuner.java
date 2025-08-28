package org.firstinspires.ftc.teamcode.OpModes.SpeediTuningAndTesting.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;

@Config
@TeleOp
public class ChassisPIDTuner extends LinearOpMode {
    MecanumDrive drive;
    enum State {
        DRIVING,
        AUTO
    }

    State state;
    Pose startPose = new Pose(0, 0, Math.toRadians(0));

    public static double xTargetPos = 150, yTargetPos = 30, angleTargetPos = 90;//TODO Change how you want
    Pose targetPos = new Pose(xTargetPos, yTargetPos, Math.toRadians(angleTargetPos));
    int traj = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new MecanumDrive(hardwareMap, startPose, telemetry, true);
        state = State.AUTO;

        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            switch (state) {
                case DRIVING:
                    if (gamepad1.b) {
                        targetPos = new Pose(xTargetPos, yTargetPos, Math.toRadians(angleTargetPos));
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
                    drive.setTargetPose(targetPos);
                    traj = 0;
                    while (opModeIsActive()) {
                        switch (traj) {
                            case 0:
                                if (drive.isDone()) {
                                    traj++;
                                    drive.setTargetPose(startPose);
                                }
                                break;
                        }
                        if (drive.isDone() && traj == 1)
                            break;
                        drive.update();
                        if (gamepad1.a) {
                            state = State.DRIVING;
                            drive.setRunMode(MecanumDrive.RunMode.MANUAL);
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
