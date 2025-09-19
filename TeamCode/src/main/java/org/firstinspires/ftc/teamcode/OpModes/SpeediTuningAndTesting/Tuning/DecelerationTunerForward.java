package org.firstinspires.ftc.teamcode.OpModes.SpeediTuningAndTesting.Tuning;

import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.GoToPointConstants.shouldUsePhysicalBraking;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.GoToPointConstants.velocityThreshold;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.MecanumChassisConstants.forwardChassisMaxVelocity;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Follower.SpeediDrive;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;

@Config
@TeleOp(name = "Deceleration Tuner Forward")
public class DecelerationTunerForward extends LinearOpMode {

    FtcDashboard dash;

    SpeediDrive drive;

    //public static double accelerationTime = 900;//MS
    public static double toleranceForVelocity = 20;//CM/S TODO to adjust if not enough space to accelerate(Note the higher it gets the worst the result is)
    private boolean stopped = false;
    private double velocityAtStop;

    private double deceleration;
    private double deltaTime;

    int step = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        dash = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());
        shouldUsePhysicalBraking = true;
        drive = new SpeediDrive(hardwareMap, new Pose(), telemetry, true);
        drive.setRunMode(SpeediDrive.RunMode.MANUAL);
        waitForStart();

        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.startTime();

        long startTime = System.currentTimeMillis();
        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            switch (step) {
                case 0:
                    if (drive.localizer.getVelocity().getY() <= forwardChassisMaxVelocity - toleranceForVelocity) {
                        drive.motors.setMotorPower(1, 1, 1, 1);
                    } else{
                        step++;
                        startTime = System.currentTimeMillis();
                        velocityAtStop = drive.localizer.getVelocity().getY();
                        drive.motors.setMotorPower(0, 0, 0, 0);
                    }
                    break;
                case 1:
                    if (drive.localizer.getVelocity().getMagnitude() <= velocityThreshold) {
                        step++;
                        deltaTime = (System.currentTimeMillis() - startTime) / 1000.0;
                        deceleration = forwardChassisMaxVelocity / deltaTime;
                    }
                    break;
            }

            telemetry.addData("pose", drive.localizer.getPoseEstimate());
            telemetry.addData("Step", step);
            telemetry.addData("yDeceleration", deceleration);
            telemetry.addData("Delta time", deltaTime);
            telemetry.addData("Velocity at stop", velocityAtStop);
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