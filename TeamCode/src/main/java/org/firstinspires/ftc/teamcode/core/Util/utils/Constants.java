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
        public static boolean shouldReverseLateralEncoder = false;
        public static boolean shouldReverseForwardEncoder = true;
        public static final double cmPerTickForward = 1.00 / (19.89436789f / 10); //TODO ADJUST THIS FOR PinpointLocalizer in case you use anything else than goBilda encoders
        public static final double cmPerTickLateral = 1.00 / (19.89436789f / 10);


        //TODO For classic TwoWheelLocalizer:
        public static double perpXTicks = 0.0; // x position of the perpendicular encoder (in tick units)
        public static double parYTicks = 0.0; // y position of the parallel encoder (in tick units)
        public static RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        public static RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;


        //TODO For PinPointLocalizer:
        public static GoBildaPinpointDriver.GoBildaOdometryPods typeOfEncoders = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
        public static double perpXEncoderForwardDistanceToCenterOfRotation = 30.0; //in mms
        public static double parYEncoderLateralDistanceToCenterOfRotation = 5.0;//in mms
    }

    @Config
    public static class GoToPointConstants {
        public static boolean shouldUsePhysicalBraking = true;//TODO IF THIS IS TRUE DO NOT ADD kD TO THE tPIDCoeff_GoToPoint AND ADJUST (xDeceleration and yDeceleration) INSTEAD
        public static double lateralDeceleration = 247.58438571923446;
        public static double forwardDeceleration = 270.13739836707487; //Deceleration for velocity-based stopping


        public static PIDCoefficients
                tPIDCoeff_GoToPoint = new PIDCoefficients(0.13, 0, 0.0),
                hPIDCoeff_GoToPoint = new PIDCoefficients(1, 0, 0.005);

        public static boolean useFinalAdj = true;
        public static PIDCoefficients
                tPIDCoeff_finalAdj = new PIDCoefficients(0.1, 0, 0.005),
                hPIDCoeff_finalAdj = new PIDCoefficients(0.7, 0, 0.0075);


        public static double velocityThreshold = 7; //velocity treshold for knowing when the robot is stopped quicker than becoming 0
        public static boolean holdFinalPoint = true;
    }


    @Config
    public static class MecanumChassisConstants {
        public static boolean shouldReverseLeftForwardMotor = false;
        public static boolean shouldReverseRightForwardMotor = true;
        public static boolean shouldReverseLeftBackMotor = false;
        public static boolean shouldReverseRightBackMotor = true;

        public static final double lateralChassisMaxVelocity = 267.5613572791492;
        public static final double forwardChassisMaxVelocity = 294.10969518367864;

        public static double[] minPowersToOvercomeStaticFriction = new double[]{ //TODO Copy values with voltage correction
                0.2054365061678371,// leftFront
                0.2253323134198863,// leftBack
                0.20239842553744758,// rightFront
                0.22502940962152113// rightBack
        };
        public static double minPowerToOvercomeKineticFriction = 0.1309;
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