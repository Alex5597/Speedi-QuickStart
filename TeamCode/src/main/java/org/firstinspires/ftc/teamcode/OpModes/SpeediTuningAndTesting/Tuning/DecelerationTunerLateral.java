package org.firstinspires.ftc.teamcode.OpModes.SpeediTuningAndTesting.Tuning;

import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.GoToPointConstants.velocityThreshold;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.MecanumChassisConstants.lateralChassisMaxVelocity;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Follower.GoToPoint;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;

@Config
@TeleOp(name = "Deceleration Tuner Lateral")
public class DecelerationTunerLateral extends LinearOpMode {

    FtcDashboard dash;

    GoToPoint drive;
    ElapsedTime timer = new ElapsedTime();

    public static double accelerationTime = 1500;//MS
    private boolean stopped = false;
    private double velocityAtStop;

    private double deceleration;
    private double deltaTime;
    int step = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());
        drive = new GoToPoint(hardwareMap, new Pose(), telemetry, true);
        drive.setRunMode(GoToPoint.RunMode.MANUAL);
        waitForStart();

        timer.reset();

        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.startTime();
        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            switch (step) {
                case 0:
                    if (timer.milliseconds() <= accelerationTime) {
                        drive.motors.setMotorPower(1, -1, -1, 1);
                    } else if (timer.milliseconds() >= 500) {
                        step++;
                        timer.reset();
                        velocityAtStop = drive.localizer.getVelocity().getX();
                        drive.motors.setMotorPower(0, 0, 0, 0);
                    }
                    break;
                case 1:
                    if (drive.localizer.getVelocity().getMagnitude() <= velocityThreshold) {
                        step++;
                        deltaTime = timer.seconds();
                        deceleration = lateralChassisMaxVelocity / deltaTime;
                    }
                    break;
            }

            telemetry.addData("pose", drive.localizer.getPoseEstimate());
            telemetry.addData("Step", step);
            telemetry.addData("Timer", timer.seconds());
            telemetry.addData("xDeceleration", deceleration);
            telemetry.addData("Delta time", deltaTime);
            telemetry.addData("Velocity at stop", velocityAtStop);
            telemetry.addData("Time since stop", timer.seconds());
            telemetry.addData("Stopped", stopped);
            telemetry.addData("Velocity x", drive.localizer.getVelocity().getX());
            telemetry.addData("Velocity y", drive.localizer.getVelocity().getY());
            telemetry.addData("Imu angle", drive.getCurrentPos().getHeading(AngleUnit.DEGREES));
            telemetry.addData("Hz", 1.0 / loopTimer.seconds());
            loopTimer.reset();

            telemetry.update();
        }
    }
}