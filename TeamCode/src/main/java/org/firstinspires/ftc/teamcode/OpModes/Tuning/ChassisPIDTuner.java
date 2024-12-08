package org.firstinspires.ftc.teamcode.OpModes.Tuning;

import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.WAIT_TIME_VARIABLE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;
import org.firstinspires.ftc.teamcode.core.Util.Math.Vector;

import java.util.LinkedList;
import java.util.Queue;

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

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new MecanumDrive(hardwareMap, startPose, telemetry, true);
        state = State.AUTO;
        Queue<Pose> targetPositions = new LinkedList<>();

        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            switch (state) {
                case DRIVING:
                    if (gamepad1.b) {
                        targetPos = new Pose(xTargetPos, yTargetPos, Math.toRadians(angleTargetPos));
                        targetPositions.add(targetPos);
                        targetPositions.add(new Pose(WAIT_TIME_VARIABLE, 500));
                        targetPositions.add(startPose);
                        targetPositions.add(new Pose(WAIT_TIME_VARIABLE, 500));
                        drive.resetPosition(startPose);
                        state = State.AUTO;
                    }
                    drive.motors.drive(gamepad1);
                    drive.update();
                    drive.currentPosTelemetry(true);
                    drive.PinPointErrorTelemetry(false);
                    break;
                case AUTO:
                    targetPositions.add(targetPos);
                    targetPositions.add(new Pose(WAIT_TIME_VARIABLE, 500));
                    targetPositions.add(startPose);
                    targetPositions.add(new Pose(WAIT_TIME_VARIABLE, 500));
                    drive.setTargetsList(targetPositions);

                    while (opModeIsActive()) {
                        drive.update();
                        if (gamepad1.a) {
                            state = State.DRIVING;
                            drive.stop();
                            break;
                        }
                        if (drive.isDone())
                            break;
                        telemetry.addData("Velocity", drive.localizer.getVelocity().getMagnitude());
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
