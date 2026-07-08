package org.firstinspires.ftc.teamcode.OpModes.SpeediTuningAndTesting.Tuning;

import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LQRSplineConstants.forwardKA;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LQRSplineConstants.forwardKV;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LQRSplineConstants.headingKA;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LQRSplineConstants.headingKV;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LQRSplineConstants.strafeKA;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.LQRSplineConstants.strafeKV;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Solves the LQR Riccati equation on the robot from the drive model measured by
 * "LQR Drive Model Tuner" (kV and kA in Constants.LQRSplineConstants must already be filled in).
 * <p>
 * The robot does NOT move: change the Q/R weights live on the dashboard, watch the K gains update,
 * then copy them into Constants.LQRSplineConstants.
 * <p>
 * Intuition for the weights: qPosition = how much a position error hurts, qVelocity = how much a
 * velocity error hurts, rEffort = how expensive motor power is. Bigger q / smaller r = more
 * aggressive gains.
 */
@Config
@TeleOp(name = "LQR Gain Calculator")
public class LQRGainCalculator extends LinearOpMode {
    public static double loopTimeSeconds = 0.02;//average control loop period of your robot
    public static double qPositionXY = 1.0;
    public static double qVelocityXY = 0.05;
    public static double rEffortXY = 2000;
    public static double qPositionHeading = 10;
    public static double qVelocityHeading = 0.5;
    public static double rEffortHeading = 5;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        while (!isStopRequested()) {
            double[] along = solveK(forwardKV, forwardKA, qPositionXY, qVelocityXY, rEffortXY, loopTimeSeconds);
            double[] cross = solveK(strafeKV, strafeKA, qPositionXY, qVelocityXY, rEffortXY, loopTimeSeconds);
            double[] head = solveK(headingKV, headingKA, qPositionHeading, qVelocityHeading, rEffortHeading, loopTimeSeconds);

            telemetry.addLine("Copy these into Constants.LQRSplineConstants:");
            telemetry.addData("alongKPosition", along[0]);
            telemetry.addData("alongKVelocity", along[1]);
            telemetry.addData("crossKPosition", cross[0]);
            telemetry.addData("crossKVelocity", cross[1]);
            telemetry.addData("headingKPosition", head[0]);
            telemetry.addData("headingKVelocity", head[1]);
            telemetry.addLine("");
            telemetry.addLine("Tune live from the dashboard: bigger qPosition/qVelocity or smaller rEffort = more aggressive robot");
            if (forwardKA <= 1e-9 || strafeKA <= 1e-9 || headingKA <= 1e-9)
                telemetry.addLine("WARNING: a kA is 0, run the LQR Drive Model Tuner first!");
            telemetry.update();
            sleep(100);
        }
    }

    /**
     * Discrete LQR for one axis with state [position error, velocity error] and the model
     * power = kV*v + kA*a  ->  vDot = (power - kV*v)/kA.
     * Iterates the Riccati equation to convergence and returns K = {kPosition, kVelocity}.
     */
    private double[] solveK(double kV, double kA, double qPos, double qVel, double r, double dt) {
        if (kA <= 1e-9 || r <= 1e-9) return new double[]{0, 0};
        double a11 = 1, a12 = dt;
        double a21 = 0, a22 = 1 - dt * kV / kA;
        double b1 = 0, b2 = dt / kA;

        double p11 = qPos, p12 = 0, p22 = qVel;
        for (int i = 0; i < 10000; i++) {
            double pb1 = p11 * b1 + p12 * b2;
            double pb2 = p12 * b1 + p22 * b2;
            double s = r + b1 * pb1 + b2 * pb2;
            double k1 = (pb1 * a11 + pb2 * a21) / s;
            double k2 = (pb1 * a12 + pb2 * a22) / s;

            double pa11 = p11 * a11 + p12 * a21, pa12 = p11 * a12 + p12 * a22;
            double pa21 = p12 * a11 + p22 * a21, pa22 = p12 * a12 + p22 * a22;
            double apa11 = a11 * pa11 + a21 * pa21, apa12 = a11 * pa12 + a21 * pa22;
            double apa21 = a12 * pa11 + a22 * pa21, apa22 = a12 * pa12 + a22 * pa22;
            double apb1 = a11 * pb1 + a21 * pb2, apb2 = a12 * pb1 + a22 * pb2;

            double n11 = qPos + apa11 - apb1 * k1;
            double n22 = qVel + apa22 - apb2 * k2;
            double n12 = ((apa12 - apb1 * k2) + (apa21 - apb2 * k1)) / 2.0;

            double diff = Math.abs(n11 - p11) + Math.abs(n12 - p12) + Math.abs(n22 - p22);
            p11 = n11;
            p12 = n12;
            p22 = n22;
            if (diff < 1e-12) break;
        }

        double pb1 = p11 * b1 + p12 * b2;
        double pb2 = p12 * b1 + p22 * b2;
        double s = r + b1 * pb1 + b2 * pb2;
        return new double[]{(pb1 * a11 + pb2 * a21) / s, (pb1 * a12 + pb2 * a22) / s};
    }
}
