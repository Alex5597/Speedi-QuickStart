package org.firstinspires.ftc.teamcode.OpModes.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;
import org.firstinspires.ftc.teamcode.core.Util.Math.Vector;


@Config
@TeleOp(name = "Deceleration Tuner Forward")
public class DecelerationTunerForward extends LinearOpMode {

    FtcDashboard dash;

    MecanumDrive drive;
    ElapsedTime timer = new ElapsedTime();

    public static double accelerationTime = 1000;//MS
    private boolean stopped = false;
    private double velocityAtStop;

    private double deceleration;
    private double deltaTime;

    int step = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        drive = new MecanumDrive(hardwareMap, new Pose(0, 0, 0), telemetry,true);
        drive.setRunMode(MecanumDrive.RunMode.MANUAL);
        waitForStart();

        timer.reset();

        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.startTime();

        long startTime = System.currentTimeMillis();
        while (opModeIsActive() && !isStopRequested()) {

            switch (step) {
                case 0:
                    if (System.currentTimeMillis() - startTime <= accelerationTime) {
                        drive.setSpeedVector(new Vector(0, 1, 0));
                        drive.updatePowerVector();
                    } else {
                        step++;
                        timer.reset();
                        velocityAtStop = drive.localizer.getVelocity().getY();
                        drive.setSpeedVector(new Vector(0, 0, 0));
                        drive.updatePowerVector();
                    }
                    break;
                case 1:
                    if (drive.localizer.getVelocity().getMagnitude() <= 0.001) {
                        step++;
                        deltaTime = timer.seconds();
                        deceleration = velocityAtStop / deltaTime;
                    }
                    break;
            }

            drive.update();

            telemetry.addData("pose", drive.localizer.getPoseEstimate());
            telemetry.addData("Step", step);
            telemetry.addData("Timer", timer.seconds());
            telemetry.addData("yDeceleration", deceleration);
            telemetry.addData("Delta time", deltaTime);
            telemetry.addData("Velocity at stop", velocityAtStop);
            telemetry.addData("Time since stop", timer.seconds());
            telemetry.addData("Stopped", stopped);
            telemetry.addData("Velocity x", drive.localizer.getVelocity().getX());
            telemetry.addData("Velocity y", drive.localizer.getVelocity().getY());
            telemetry.addData("Imu angle", drive.getCurrentPos().getHeading());
            telemetry.addData("Hz", 1.0 / loopTimer.seconds());
            loopTimer.reset();

            telemetry.update();
        }
    }
}