/*
Configuration

Control hub:
Motors:
0 - LFM (Parallel encoder)
1 - RFM (Perpendicular encoder)
2 - RBM
3 - LBM

Servo motors:
0 - LSR// Linear slides right
1 - WI // wrist intake
2 - I // intake
3 - AI // Arm intake
5 - L1// Unul dintre leduri
I2C:
0-imu //imu sensor
1- odo //pinpoint
2- IS //color sensor intake


Expansion hub:
Motors:
0 - LM //liftMotor (Lift encoder)
1 - ASCL // ascendLeft
2 - ASCR // ascendRight
Servo motors:
0 - WO// wrist outtake
1 - LSL// linear slides left
2 - C //claw
3 - AO //arm outtake
5 - L2//Alt led
*/

package org.firstinspires.ftc.teamcode.core.Util.utils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.core.Util.Hardware.GoBildaPinpointDriver;

@Config
public class Constants {
    public static boolean useDashboard = true;//TODO Should be false after the autos is done so loop time is better
    public static boolean disableWarningErrors = false; //TODO Should be true after the autos is done so no crashes happen during comps
    public static double robotWidthInCMs = 40;//LEFT TO RIGHT DISTANCE(positive always)
    public static double robotLengthInCMs = 40;//FRONT TO BACK DISTANCE(positive always)

    @Config
    public static class LocalizerConstants {
        //SPEEDI frame: X right positive, Y forward positive, heading CLOCKWISE positive
        //checked with "PinPointLocalizerTest": drive forward -> Y up, right -> X up, turn right -> heading up
        public static boolean shouldReverseLateralEncoder = false;
        public static boolean shouldReverseForwardEncoder = true;
        public static final double cmPerTickForward = 1.00 / (19.89436789f / 10); //TODO ADJUST THIS FOR PinpointLocalizer in case you use anything else than goBilda encoders
        public static final double cmPerTickLateral = 1.00 / (19.89436789f / 10);


        //TODO For classic TwoWheelLocalizer:
        public static double perpXTicks = 0.0; // x position of the perpendicular encoder (in tick units)
        public static double parYTicks = 0.0; // y position of the parallel encoder (in tick units)
        public static RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        public static RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;


        //TODO For PinPointLocalizer: measure with a ruler or run "Odometry Pod Offsets Tuner"
        public static GoBildaPinpointDriver.GoBildaOdometryPods typeOfEncoders = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
        //how far SIDEWAYS (mm) from the robot's center of rotation the X (forward) pod is: LEFT of center is positive, RIGHT is negative
        public static double xPodOffsetInMM = 5.0;
        //how far FORWARD (mm) from the robot's center of rotation the Y (strafe) pod is: FORWARD of center is positive, BACKWARDS is negative
        public static double yPodOffsetInMM = 30.0;
    }

    @Config
    public static class GoToPointConstants {
        public static boolean shouldUsePhysicalBraking = true;//TODO IF THIS IS TRUE DO NOT ADD kD TO THE tPIDCoeff_GoToPoint AND ADJUST (xDeceleration and yDeceleration) INSTEAD

        //measured by "Braking Deceleration Tuner", then fine tuned LIVE in "Test Predicted Pose"
        //until the predicted stop point matches where the robot actually stops
        public static double lateralDeceleration = 247.58438571923446;
        public static double forwardDeceleration = 270.13739836707487; //Deceleration for velocity-based stopping

        //tuned by hand with "ChassisPIDTuner" (dashboard): raise p until the robot reaches the target fast without oscillating
        public static PIDCoefficients
                tPIDCoeff_GoToPoint = new PIDCoefficients(0.13, 0, 0.0),
                hPIDCoeff_GoToPoint = new PIDCoefficients(1, 0, 0.005);

        //slower, precise PID used near the target for the final adjustment
        public static boolean useFinalAdj = true;
        public static PIDCoefficients
                tPIDCoeff_finalAdj = new PIDCoefficients(0.1, 0, 0.005),
                hPIDCoeff_finalAdj = new PIDCoefficients(0.7, 0, 0.0075);


        public static double velocityThreshold = 7; //velocity treshold for knowing when the robot is stopped quicker than becoming 0
        public static boolean holdFinalPoint = true;
    }


    @Config
    public static class MecanumChassisConstants {
        //set with "MotorConfigTest": flip a motor if it spins backwards there
        public static boolean shouldReverseLeftForwardMotor = false;
        public static boolean shouldReverseRightForwardMotor = true;
        public static boolean shouldReverseLeftBackMotor = false;
        public static boolean shouldReverseRightBackMotor = true;

        //measured by "MinimumPowerToOvercomeFrictionDrivetrainTuner" (copy the values WITH voltage correction)
        public static double[] minPowersToOvercomeStaticFriction = new double[]{
                0.2054365061678371,// leftFront
                0.2253323134198863,// leftBack
                0.20239842553744758,// rightFront
                0.22502940962152113// rightBack
        };
        //measured by "Kinetic Kstatic Tuner"
        public static double minPowerToOvercomeKineticFriction = 0.1309;
        //measured by "SWITCH_FROM_STATIC_TO_KINETIC_FRICTIONTuner"
        public static final int SWITCH_FROM_STATIC_TO_KINETIC_FRICTION = 92;//In MS


        public static double lateralMultiplier = 2.2;
        /// In case u want slower heading rate of change at GoToPoint
        public static final double Lateral = lateralMultiplier;
        /// In case u want slower lateral rate of change(can also correct strafing imperfections)
        public static double forwardMultiplier = 1;
        public static final double Forward = forwardMultiplier;
        /// In case u want slower heading rate of change at GoToPoint
        public static double headingMultiplier = 1;
        public static final double Heading = headingMultiplier;

        public static void resetMultipliers() {
            headingMultiplier = Heading;
            lateralMultiplier = Lateral;
            forwardMultiplier = Forward;
        }
    }

    @Config
    public static class FollowerConstants {
        public final static int resolution = 1000;
        public static double TotalMassOfRobot = 9.5; //In KG
        public static double CentripetalScalingFactor = 0.00032;
        public static boolean shouldBrake = true;
        public static PIDCoefficients
                tPIDCoeff_SplineFollower = new PIDCoefficients(0.11, 0, 0.001),
                hPIDCoeff_SplineFollower = new PIDCoefficients(0.5, 0, 0.001);
    }

    @Config
    public static class LQRSplineConstants {
        //Drive model, measured by "LQR Drive Model Tuner" (one run per axis): power = kS*sign(v) + kV*v + kA*a
        public static double forwardKS = 0.13;//axis 0
        public static double forwardKV = 0.0034;
        public static double forwardKA = 0.0010;
        public static double strafeKS = 0.15;//axis 1
        public static double strafeKV = 0.0038;
        public static double strafeKA = 0.0012;
        public static double headingKS = 0.10;//axis 2
        public static double headingKV = 0.25;
        public static double headingKA = 0.03;

        //Motion profile. The max velocity is computed from the model above (forward and strafe are
        //different automatically), so there is no separate max velocity constant to tune
        public static double profilePowerBudget = 0.9;//THE speed knob: fraction of full power the path planning uses, the rest stays as headroom for corrections
        public static double maxAcceleration = 250;//cm/s^2, copy ~80% of the value from "LQR Drive Model Tuner"
        public static double maxDeceleration = 300;//cm/s^2, copy ~80% of the value from "LQR Drive Model Tuner"

        //LQR state feedback gains, computed by "LQR Gain Calculator" from the model above
        public static double alongKPosition = 0.021;
        public static double alongKVelocity = 0.0055;
        public static double crossKPosition = 0.021;
        public static double crossKVelocity = 0.0057;
        public static double headingKPosition = 1.30;
        public static double headingKVelocity = 0.23;

        //Small PID trim on top of the LQR for residual errors, keep the gains low (the I term removes steady state error)
        public static PIDCoefficients xyPIDCoeff_LQR = new PIDCoefficients(0.004, 0.008, 0);
        public static PIDCoefficients hPIDCoeff_LQR = new PIDCoefficients(0.2, 0.4, 0);

        //Behavior
        public static double slowdownDemandThreshold = 0.4;//fraction of correction saturation before the robot starts slowing down (higher = faster but sloppier in curves)
        public static double minPathSpeedScale = 0.35;//speed floor when the corrections are saturated
        public static double alongLagTolerance = 8;//cm the robot may fall behind the profile before it starts pausing
        public static double maxCorrectionPower = 0.55;
        public static double maxHeadingPower = 0.45;
        public static double frictionVelocityDeadband = 5;//cm/s under which the kS term fades to 0
        public static double frictionOmegaDeadband = 0.2;//rad/s under which the heading kS term fades to 0

        //isFinished tolerances
        public static double positionTolerance = 3;//cm
        public static double headingTolerance = 3;//degrees
        public static double velocityTolerance = 10;//cm/s
        public static double angularVelocityTolerance = 8;//degrees/s
    }

    public static final double WAIT_TIME_VARIABLE = 99999999.991; //IGNORA

    @Config
    public static class DeviceNames {
        public static String leftFrontMotorName = "LFM";
        public static String leftBackMotorName = "LBM";
        public static String rightFrontMotorName = "RFM";
        public static String rightBackMotorName = "RBM";
        public static String pinPointName = "odo";
    }
}
