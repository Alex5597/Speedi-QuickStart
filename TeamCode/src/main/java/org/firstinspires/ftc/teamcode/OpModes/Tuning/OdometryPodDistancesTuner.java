package org.firstinspires.ftc.teamcode.OpModes.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Localizer.Localizer;
import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Localizer.PinPointLocalizer;
import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Localizer.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;
import org.firstinspires.ftc.teamcode.core.Util.Math.Vector;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

//NOT WORKING
@Disabled
@Config
@TeleOp
public class OdometryPodDistancesTuner extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime();
    MecanumDrive drive;
    PinPointLocalizer localizer;
    public static double timeToRotate_MS = 10000;//MS
    private double lastAngle = 0, lastTime = 0;
    private final List<Pose> rXCoordinates = new ArrayList<>();
    private final List<Pose> rYCoordinates = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        localizer = new PinPointLocalizer(hardwareMap, new Pose(), telemetry);
        drive = new MecanumDrive(hardwareMap, new Pose(), telemetry, true, true);

        waitForStart();
        drive.motors.setMotorPower(0.5, 0.5, -0.5, -0.5);
        timer.reset();
        while (timeToRotate_MS >= timer.milliseconds() || isStopRequested()) {
            double angle = localizer.getPoseEstimate().getHeading();
            double w = 0;
            if (timer.seconds() - lastTime != 0)
                w = (angle - lastAngle) / (timer.seconds() - lastTime); // rad/s

            rXCoordinates.add(new Pose(w, localizer.odo.getEncoderX() / (timer.seconds() - lastTime)));
            rYCoordinates.add(new Pose(w, localizer.odo.getEncoderY() / (timer.seconds() - lastTime)));


            telemetry.addData("Angular velocity", w);
            telemetry.addData("xVelo", localizer.odo.getEncoderX() / (timer.seconds() - lastTime));
            telemetry.addData("yVelo", localizer.odo.getEncoderY() / (timer.seconds() - lastTime));
            telemetry.update();
            lastAngle = angle;
            lastTime = timer.seconds();
            localizer.update();
        }
        drive.stop();
        double slopeX = calculateTheilSenSlope(rXCoordinates), slopeY = calculateTheilSenSlope(rYCoordinates);

        while (opModeIsActive()) {
            telemetry.addData("perpXTicks", slopeX);
            telemetry.addData("parYTicks", slopeY);
            telemetry.update();
        }
    }

    public static double calculateTheilSenSlope(List<Pose> poses) {
        List<Double> slopes = new ArrayList<>();

        // Calculate slopes between all unique pairs of points
        for (int i = 0; i < poses.size(); i++) {
            for (int j = i + 1; j < poses.size(); j++) {
                double x1 = poses.get(i).getX();
                double y1 = poses.get(i).getY();
                double x2 = poses.get(j).getX();
                double y2 = poses.get(j).getY();

                if (x2 != x1) { // Avoid division by zero
                    slopes.add((y2 - y1) / (x2 - x1));
                }
            }
        }

        // Sort the slopes
        Collections.sort(slopes);

        // Return the median slope
        int mid = slopes.size() / 2;
        if (slopes.size() % 2 == 0) {
            return (slopes.get(mid - 1) + slopes.get(mid)) / 2.0;
        } else {
            return slopes.get(mid);
        }
    }
}
