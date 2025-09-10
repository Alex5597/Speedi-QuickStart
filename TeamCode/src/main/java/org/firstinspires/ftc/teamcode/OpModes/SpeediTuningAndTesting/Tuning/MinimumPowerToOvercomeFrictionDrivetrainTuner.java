package org.firstinspires.ftc.teamcode.OpModes.SpeediTuningAndTesting.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;
import org.firstinspires.ftc.teamcode.core.Util.utils.Globals;

@TeleOp
public class MinimumPowerToOvercomeFrictionDrivetrainTuner extends LinearOpMode {
    double[] sums = new double[4];
    double[] sumsV = new double[4];
    int iterations = 5;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose(), telemetry,true);
        drive.setRunMode(MecanumDrive.RunMode.MANUAL);
        double[] minPowersToOvercomeFriction = new double[4];

        Pose robotPose;
        drive.motors.resetMinPowersToOvercomeFriction();

        waitForStart();

        for (int i = 0; i < 4; i++) {

            for (int a = 0; a < iterations; a++) {
                drive.resetPosition(new Pose());
                long start = System.currentTimeMillis();
                for (double j = 0; j < 1; j = (double) (System.currentTimeMillis() - start) / (15000.0)) {
                    drive.update();
                    drive.motors.motors.get(i).setTargetPower(j);

                    robotPose = drive.getLocalizerInstance().getPoseEstimate();
                    if (Math.abs(robotPose.getX(DistanceUnit.CM)) > 1 || Math.abs(robotPose.getY(DistanceUnit.CM)) > 1 || Math.abs(robotPose.getHeading(AngleUnit.RADIANS)) > Math.toRadians(1.3)) {
                        minPowersToOvercomeFriction[i] = j;
                        break;
                    }
                    telemetry.addData(drive.motors.motors.get(i).name + " current power: ", j);
                    telemetry.update();
                }

                drive.motors.motors.get(i).setTargetPower(0.0);

                sums[i] += minPowersToOvercomeFriction[i] * (12.0 / Globals.voltage);
                sumsV[i] += minPowersToOvercomeFriction[i];

                long waitStart = System.currentTimeMillis();
                while (System.currentTimeMillis() - waitStart < 1000) {
                    drive.update();
                }
            }

            telemetry.addData(drive.motors.motors.get(i).name + " AVERAGE min power with voltage correction", sums[i] / iterations + "");
            telemetry.update();
        }

        while (opModeIsActive()) {
            for (int i = 0; i < 4; i++) {
                telemetry.addData(drive.motors.motors.get(i).name + " fara voltaj: ", sumsV[i] / iterations);
                telemetry.addData(drive.motors.motors.get(i).name + " cu voltaj: ", sums[i] / iterations);
            }
            telemetry.update();
        }
    }
}