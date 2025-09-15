package org.firstinspires.ftc.teamcode.OpModes.SpeediTuningAndTesting.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Follower.GoToPoint;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;
import org.firstinspires.ftc.teamcode.core.Util.utils.Constants;

@Config
@TeleOp
public class SplinePIDTuner extends LinearOpMode {
    GoToPoint drive;

    enum State {
        DRIVING,
        AUTO
    }

    State state;
    Pose startPose = new Pose();

    public static double xTargetPos = 0, yTargetPos = 0, angleTargetPos = 0;//TODO Change how you want (be careful of the tolerance)
    Pose targetPos = new Pose(xTargetPos, yTargetPos, DistanceUnit.CM, angleTargetPos, AngleUnit.DEGREES);
    int traj = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new GoToPoint(hardwareMap, startPose, telemetry, true);
        state = State.AUTO;

        waitForStart();

        while (opModeIsActive()) {
            switch (state) {
                case DRIVING:
                    if (gamepad1.b) {
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
                    drive.setTargetPose(targetPos,true);
                    Constants.GoToPointConstants.holdFinalPoint = true;
                    drive.setRunMode(GoToPoint.RunMode.CalibrateSplinePID);
                    while (opModeIsActive()) {
                        drive.update();
                        if (gamepad1.a) {
                            state = State.DRIVING;
                            drive.setRunMode(GoToPoint.RunMode.MANUAL);
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
