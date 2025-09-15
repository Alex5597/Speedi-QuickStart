package org.firstinspires.ftc.teamcode.OpModes.SpeediTuningAndTesting.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Follower.SpeediDrive;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;

import java.util.ArrayDeque;
import java.util.Deque;

@Disabled
@Config
@TeleOp
public class MaxForwardAcceleration extends LinearOpMode {

    // ===== Dashboard-tunable params =====
    public static double maxAllowedTimeInSecs = 1.3; // accel phase duration
    public static double settleSecs = 0.5;           // let filters settle before run
    public static int samplePeriodMs = 20;           // ~50 Hz sampling
    public static int fitWindow = 20;                // samples used for slope (~0.4 s)

    SpeediDrive drive;
    double maxAccel = 0.0;

    private static class Sample {
        final double t;  // seconds
        final double v;  // forward speed (units/s)
        Sample(double t, double v) { this.t = t; this.v = v; }
    }
    private final Deque<Sample> window = new ArrayDeque<>();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SpeediDrive(hardwareMap, new Pose(), telemetry, true, true);

        telemetry.addLine("MaxForwardAcceleration: place robot on a long, straight, grippy lane.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // --- settle ---
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive() && timer.seconds() < settleSecs) {
            drive.update();
            sleep(10);
        }

        // --- accelerate ---
        timer.reset();
        window.clear();
        maxAccel = 0.0;

        // full forward (adjust if your API differs)
        drive.motors.setMotorPower(1, 1, 1, 1);

        long lastSampleNs = System.nanoTime();
        while (opModeIsActive() && timer.seconds() <= maxAllowedTimeInSecs) {
            // keep loop light and consistent
            if (System.nanoTime() - lastSampleNs < samplePeriodMs * 1_000_000L) {
                drive.update(); // make sure localizer advances even between samples
                continue;
            }
            lastSampleNs = System.nanoTime();

            // update first, then read
            drive.update();

            double t = timer.seconds();

            // forward (along-heading) speed = projection of field velocity on robot heading
            Pose pose = drive.localizer.getPoseEstimate();
            double heading = pose.getHeading(AngleUnit.RADIANS);
            double vx = drive.localizer.getVelocity().getX();
            double vy = drive.localizer.getVelocity().getY();
            double vAlong = vx * Math.cos(heading) + vy * Math.sin(heading);

            addSample(t, vAlong);
            double a = slope(window); // robust dv/dt over window
            if (a > maxAccel) maxAccel = a;

            telemetry.addData("t (s)", "%.3f", t);
            telemetry.addData("vAlong (u/s)", "%.3f", vAlong);
            telemetry.addData("a_window (u/s^2)", "%.3f", a);
            telemetry.addData("a_peak (u/s^2)", "%.3f", maxAccel);
            telemetry.update();
        }

        // --- stop & report ---
        drive.motors.setMotorPower(0, 0, 0, 0);
        drive.update();

        while (opModeIsActive()) {
            telemetry.addData("Max forward acceleration (units/s^2)", "%.3f", maxAccel);
            telemetry.addLine("Tip: use ~80% of this as your MaxForwardAcceleration constant.");
            telemetry.update();
        }
    }

    private void addSample(double t, double v) {
        window.addLast(new Sample(t, v));
        while (window.size() > fitWindow) window.removeFirst();
    }

    /** Linear regression slope dv/dt over the window (handles noise & uneven dt). */
    private static double slope(Deque<Sample> win) {
        int n = win.size();
        if (n < 3) return 0.0;
        double sumT = 0, sumV = 0, sumTT = 0, sumTV = 0;
        for (Sample s : win) {
            sumT += s.t;
            sumV += s.v;
            sumTT += s.t * s.t;
            sumTV += s.t * s.v;
        }
        double denom = n * sumTT - sumT * sumT;
        if (Math.abs(denom) < 1e-9) return 0.0;
        return (n * sumTV - sumT * sumV) / denom;
    }
}
