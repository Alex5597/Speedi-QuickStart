package org.firstinspires.ftc.teamcode.OpModes.SpeediTuningAndTesting.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Follower.SpeediDrive;
import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Localizer.PinPointLocalizer;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Rotate-in-place tuner that estimates X/Y odometry pod offsets from distance-vs-heading slopes.
 * - Works with heading wrapped to [-180, 180] degrees.
 * - Supports either CCW-positive or CW-positive heading conventions.
 *
 * Assumptions:
 *   - "X encoder" measures FORWARD (+Y) distance; "Y encoder" measures STRAFE (+X) distance.
 *   - localizer.getPoseEstimate().getHeading(AngleUnit.RADIANS) is CCW-positive (REV/FTCLib norm).
 *
 * Results shown in mm w.r.t. your chosen "tracking point" (usually robot center).
 */
@Config
@TeleOp(name = "Odometry Pod Offsets Tuner")
public class OdometryPodOffsetsTuner extends LinearOpMode {

    // ---- User-tunable config (Dashboard) ----
    public static boolean AUTO_SPIN = true;       // set false if you want to rotate by hand
    public static boolean CCW_POSITIVE = true;    // set true for standard math convention (recommended)
    public static double  TARGET_UNWRAP_DEG = 1080; // total absolute unwrapped rotation
    public static long    SAMPLE_PERIOD_MS  = 20;   // log period
    public static double  ROTATE_POWER      = 0.25; // spin power if AUTO_SPIN

    // convert your encoders to mm
    public static double MM_PER_TICK_X = 0.04712;   // forward pod (X encoder)
    public static double MM_PER_TICK_Y = 0.04712;   // strafe  pod (Y encoder)

    // ------------------------------------------------
    private SpeediDrive drive;
    private PinPointLocalizer localizer;

    private final List<Double> thetaDeg = new ArrayList<>(); // unwrapped heading (deg, sign per CCW_POSITIVE)
    private final List<Double> sFwdMm   = new ArrayList<>(); // forward pod distance (mm)
    private final List<Double> sStrMm   = new ArrayList<>(); // strafe  pod distance (mm)

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Your project-specific classes; adjust ctor args if needed
        localizer = new PinPointLocalizer(hardwareMap,
                new org.firstinspires.ftc.teamcode.core.Util.Math.Pose(), telemetry);
        drive = new SpeediDrive(hardwareMap,
                new org.firstinspires.ftc.teamcode.core.Util.Math.Pose(), telemetry, true, true);

        telemetry.addLine("Odometry Pod Offsets Tuner");
        telemetry.addLine("Press PLAY, then keep the robot centered while it rotates.");
        telemetry.addData("AUTO_SPIN", AUTO_SPIN);
        telemetry.addData("CCW_POSITIVE", CCW_POSITIVE);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // optional autorotation
        if (AUTO_SPIN) {
            // choose a direction to minimize cable twist: start CCW
            setRotatePower(+Math.abs(ROTATE_POWER)); // + = CCW, - = CW (for this method)
        }

        // --- initialize logging ---
        double lastDeg = getHeadingDeg(); // wrapped to [-180, 180], sign per CCW_POSITIVE
        double unwrappedDeg = 0.0;

        double startXTicks = localizer.odo.getEncoderX(); // forward (Y) pod
        double startYTicks = localizer.odo.getEncoderY(); // strafe  (X) pod

        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && Math.abs(unwrappedDeg) < TARGET_UNWRAP_DEG) {
            localizer.update();

            double nowDeg = getHeadingDeg();
            double dDeg = wrap180(nowDeg - lastDeg);     // incremental change (deg)
            unwrappedDeg += dDeg;
            lastDeg = nowDeg;

            double fwdMm = (localizer.odo.getEncoderX() - startXTicks) * MM_PER_TICK_X;
            double strMm = (localizer.odo.getEncoderY() - startYTicks) * MM_PER_TICK_Y;

            thetaDeg.add(unwrappedDeg);
            sFwdMm.add(fwdMm);
            sStrMm.add(strMm);

            telemetry.addData("θ unwrapped (deg)", "%.1f", unwrappedDeg);
            telemetry.addData("Forward / Strafe (mm)", "%.1f / %.1f", fwdMm, strMm);
            telemetry.addData("Samples", thetaDeg.size());
            telemetry.update();

            long sleepMs = SAMPLE_PERIOD_MS - ((long) t.milliseconds() % SAMPLE_PERIOD_MS);
            sleep(Math.max(5, sleepMs));
        }

        setRotatePower(0);

        // --- robust slopes (distance vs heading) ---
        double slopeF_mm_per_deg = theilSenSlope(thetaDeg, sFwdMm); // Δs_forward / Δθ°
        double slopeS_mm_per_deg = theilSenSlope(thetaDeg, sStrMm); // Δs_strafe  / Δθ°

        // convert to offsets (mm) depending on your heading sign convention
        // Pure rotation kinematics at point (x,y):
        //   v_y =  ω x,  v_x = -ω y   with ω positive for CCW
        // => Δs_forward = x Δθ,  Δs_strafe = -y Δθ
        // If θ is CCW-positive:  x =  slopeF*deg2rad,  y = -slopeS*deg2rad
        // If θ is CW-positive :  x = -slopeF*deg2rad, y =  slopeS*deg2rad
        double deg2rad = Math.PI / 180.0;
        double xOffsetMm, yOffsetMm;
        if (CCW_POSITIVE) {
            xOffsetMm =  slopeF_mm_per_deg / deg2rad;  // slopeF * (180/π)
            yOffsetMm = -slopeS_mm_per_deg / deg2rad;
        } else {
            xOffsetMm = -slopeF_mm_per_deg / deg2rad;
            yOffsetMm =  slopeS_mm_per_deg / deg2rad;
        }

        while (opModeIsActive()) {
            telemetry.addLine("====== RESULTS (relative to tracking point) ======");
            telemetry.addData("X pod offset  (mm, +right)",  "%.1f", xOffsetMm);
            telemetry.addData("Y pod offset  (mm, +forward)","%.1f", yOffsetMm);
            telemetry.addData("Samples", thetaDeg.size());
            telemetry.addData("Slope fwd  (mm/deg)", "%.5f", slopeF_mm_per_deg);
            telemetry.addData("Slope str  (mm/deg)", "%.5f", slopeS_mm_per_deg);
            telemetry.update();
            sleep(100);
        }
    }

    // ---- helpers ----

    /** Set rotation power: + = CCW, - = CW. Uses your SpeediDrive motor helper. */
    private void setRotatePower(double rotPower) {
        if (!AUTO_SPIN) return;
        // Adjust signs/order for your drivetrain if needed.
        // (FL, FR, RL, RR) chosen so +rotPower => CCW in-place rotation.
        double p = rotPower;
        drive.motors.setMotorPower(+p, +p, -p, -p);
    }

    /** Heading in degrees, wrapped to [-180, 180], sign per CCW_POSITIVE flag. */
    private double getHeadingDeg() {
        // Most localizers report CCW+ yaw in radians:
        double yawCCWrad = localizer.getPoseEstimate().getHeading(AngleUnit.RADIANS);
        double deg = Math.toDegrees(yawCCWrad);
        if (!CCW_POSITIVE) deg = -deg;  // flip to CW+
        return wrap180(deg);
    }

    /** Wrap degrees to [-180, 180]. */
    private static double wrap180(double deg) {
        double x = deg % 360.0;
        if (x <= -180.0) x += 360.0;
        if (x >   180.0) x -= 360.0;
        return x;
    }

    /** Robust Theil–Sen slope of y vs x (median of pairwise slopes). */
    private static double theilSenSlope(List<Double> x, List<Double> y) {
        int n = Math.min(x.size(), y.size());
        if (n < 2) return 0.0;
        List<Double> slopes = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            double xi = x.get(i), yi = y.get(i);
            for (int j = i + 1; j < n; j++) {
                double dx = x.get(j) - xi;
                if (dx != 0.0) slopes.add((y.get(j) - yi) / dx);
            }
        }
        if (slopes.isEmpty()) return 0.0;
        Collections.sort(slopes);
        int m = slopes.size();
        return (m & 1) == 1 ? slopes.get(m / 2)
                : 0.5 * (slopes.get(m / 2 - 1) + slopes.get(m / 2));
    }
}
