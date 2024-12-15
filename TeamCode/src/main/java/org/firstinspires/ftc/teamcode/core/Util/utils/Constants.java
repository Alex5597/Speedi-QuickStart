/*
Configuration

Control hub:
Motors:
0 - LFM (Parallel encoder)
1 - RFM (Perpendicular encoder)
2 - RBM
3 - LBM
Servo motors:
0 - LS// linear slides
2 - LI // left intake active?
4 - RI // right intake active?
I2C:
0-imu //imu sensor
1- odo //pinpoint
2- IS //color sensor intake


Expansion hub:
Motors:
0 - LM //liftMotor (Lift encoder)
2 - ASCL // ascendLeft
3 - ASCR // ascendRight
Servo motors:
0 - W // wrist
2 - A// servo arm
4 - IP //intake passive
5 - SB //servo bucket

I2C:
0- BS// bucket sensor
*/

package org.firstinspires.ftc.teamcode.core.Util.utils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.core.Util.Math.Vector;

@Config
public class Constants {
    public static boolean useDashboard = true;
    //Localization Constants ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
    public static final double cmPerTickForward = 1.00 / (8192.00 / (35.0 * Math.PI));
    /// FOR REV THROUGH BORE ENCODERS
    public static final double cmPerTickLateral = 1.00 / (8192.00 / (35.0 * Math.PI));
    /// FOR REV THROUGH BORE ENCODERS

    //For classic TwoWheelLocalizer:
    public static double perpXTicks = 10758.672; // x position of the perpendicular encoder (in tick units)
    public static double parYTicks = 0.0; // y position of the parallel encoder (in tick units)


    //For PinPointLocalizer:
    public static double perpXEncoderForwardDistanceToCenterOfRotation = 160.00; //in mm
    public static double parYEncoderLateralDistanceToCenterOfRotation = -14.50;//in mm


    public static double xDeceleration = 125.6468960303166, yDeceleration = 69.6536728634694469; //Deceleration for velocity-based stopping

    //Localization Constants ↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑


    //GoToPoint Constants ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
    public static double velocityThreshold = 5.0; //velocity treshold for knowing when the robot is stopped quicker than becoming 0

    @Config
    public static class DriveCorrectionCoefficients {
        public static PIDCoefficients
                xPIDCoeff = new PIDCoefficients(0.13, 0, 0.02),
                yPIDCoeff = new PIDCoefficients(0.08, 0, 0.013),
                hPIDCoeff = new PIDCoefficients(0.9, 0, 0.1);


        public static PIDCoefficients
                xPIDCoeff_finalAdj = new PIDCoefficients(0.095, 0, 0.01),
                yPIDCoeff_finalAdj = new PIDCoefficients(0.085, 0, 0.01),
                hPIDCoeff_finalAdj = new PIDCoefficients(1.2, 0, 0.35);
    }

    public static final double WAIT_TIME_VARIABLE = 99999999.991;
    //GoToPoint Constants ↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑


    //Chassis Constants ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
    private static final double xMaxVelocity = 153.03941650390624; //lateral
    private static final double yMaxVelocity = 243.7246337890625; //forward
    public static Vector frontLeftVector = new Vector(-xMaxVelocity, yMaxVelocity).scaleToMagnitude(1);

    //TODO Copy values with voltage correction
    public static double[] minPowersToOvercomeStaticFriction = new double[]{
            0.3680964613248011,// leftFront
            0.32564413631117617,// leftBack
            0.39037267673847714,// rightFront
            0.3747464238408216// rightBack
    };
    public static double minPowerToOvercomeKineticFriction = 0.187;
    public static final int SWITCH_FROM_STATIC_TO_KINETIC_FRICTION = 13;//In MS


    public static double lateralMultiplier = 1;
    /// In case u want slower lateral rate of change(can also correct strafing imperfections)
    public static double forwardMultiplier = 1;
    /// In case u want slower heading rate of change at GoToPoint
    public static double headignMultiplier = 1;
    /// In case u want slower heading rate of change at GoToPoint

    public static final double Lateral = lateralMultiplier;
    public static final double Forward = forwardMultiplier;
    public static final double Heading = headignMultiplier;
    //Chassis Constants ↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑

    public static void resetMultipliers() {
        headignMultiplier = Heading;
        lateralMultiplier = Lateral;
        forwardMultiplier = Forward;
    }

    @Config
    public static class FollowerConstants {
        public static double TotalMassOfRobot = 15.2; //(In KG)
        public static double CentripetalScalingFactor = 0;
    }

    @Config
    public static class LiftCorrectionCoefficients {
        public static PIDCoefficients liftPIDCoefficients = new PIDCoefficients(0.012, 0, 0.00013);
        public static double gravityGain = 0;
    }

    @Config
    public static class ArmPoses {
        public static double armCollectPose = 0;
        public static double armScorePose = 1;
    }

    @Config
    public static class LiftPositions {
        public static double liftScoreHighSpecimen = 2040, liftLowChamber = 510, liftHighChamber = 2600;
        public static double liftCollect = 0, liftLowBasket = 2100, liftHighBasket = 4280;
    }

    @Config
    public static class IntakePassivePositions {
        public static double intakePassiveClosePose = 0.0;
        public static double intakePassiveOpenPose = 1.0;
    }

    @Config
    public static class IntakeActivePower {
        public static double intakeCollect = 0.75;
        public static double intakeScore = -0.4;
        public static double intakeWait = 0;
    }

    @Config
    public static class LinearSlidePositions {
        public static double slidesExtendedPose = 0.3;
        public static double slidesAuxPose = 0.15;
        public static double slidesRetractedPose = 0.0;
    }

    @Config
    public static class BucketPositions {
        public static double bucketWaitPose = 0.31;
        public static double bucketHoldPose = 0.12;
    }

    @Config
    public static class WristPositions {
        public static double wristWaitPose = 0.15;
        public static double wristCollectPose = 0.950;
        public static double wristTransferPose = 0.03;
    }

    @Config
    public static class ColorValues {
        public static float TreshHold[] = {20F, 0.2F, 5F};
        public static float RedValues[] = {20F, 0.7F, 12.2F};
        public static float BlueValues[] = {220.5F, 0.75F, 15F};
        public static float YellowValues[] = {75F, 0.7F, 25F};
    }
}