package org.firstinspires.ftc.teamcode.OpModes.SpeediTuningAndTesting.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.core.Util.Algorithm.SplineGenerator.BezierSpline;
import org.firstinspires.ftc.teamcode.core.Util.Algorithm.SplineGenerator.CubicBezierCurve;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;
import org.firstinspires.ftc.teamcode.core.Util.Math.Vector;

@Config
@TeleOp
public class SplinePIDTuner extends LinearOpMode {
    MecanumDrive drive;

    enum State {
        DRIVING,
        AUTO
    }

    State state;
    Pose startPose = new Pose(-46, 15, Math.toRadians(-35));

    public static double xTargetPos = 130, yTargetPos = 0, angleTargetPos = 0;//TODO Change how you want (be careful of the tolerance)
    Pose targetPos = new Pose(xTargetPos, yTargetPos, Math.toRadians(angleTargetPos));
    int traj = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new MecanumDrive(hardwareMap, startPose, telemetry, true);
        state = State.AUTO;
        BezierSpline spline = new BezierSpline(Math.toRadians(angleTargetPos), new CubicBezierCurve(new Vector(0, 0), new Vector(0, 0), new Vector(0, 0), targetPos.toVec()));
        waitForStart();

        while (opModeIsActive()) {
            switch (state) {
                case DRIVING:
                    if (gamepad1.b) {
                        spline = new BezierSpline(Math.toRadians(angleTargetPos), new CubicBezierCurve(new Vector(0, 0), new Vector(0, 0), new Vector(0, 0), targetPos.toVec()));
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
                    drive.setSpline_withInstantHeadingChange(spline);
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
