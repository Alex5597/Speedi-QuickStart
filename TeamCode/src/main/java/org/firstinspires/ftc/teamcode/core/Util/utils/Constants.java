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
    public static boolean useFinalAdj = true;
    //Localization Constants ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
    public static final double cmPerTickForward = 0.0;
    public static final double cmPerTickLateral = 0.0;

    //For classic TwoWheelLocalizer:
    public static double perpXTicks = 0.0; // x position of the perpendicular encoder (in tick units)
    public static double parYTicks = 0.0; // y position of the parallel encoder (in tick units)


    //For PinPointLocalizer:
    public static double perpXEncoderForwardDistanceToCenterOfRotation = 0.0; //in mm
    public static double parYEncoderLateralDistanceToCenterOfRotation = 0.0;//in mm


    public static double xDeceleration = 1, yDeceleration = 1; //Deceleration for velocity-based stopping

    //Localization Constants ↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑


    //GoToPoint Constants ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
    public static double velocityThreshold = 5; //velocity treshold for knowing when the robot is stopped quicker than becoming 0 David2K9SRB was here

    @Config
    public static class DriveCorrectionCoefficients {
        public static PIDCoefficients
                tPIDCoeff_GoToPoint = new PIDCoefficients(0, 0, 0),
                xPIDCoeff_Spline = new PIDCoefficients(0, 0, 0),
                yPIDCoeff_Spline = new PIDCoefficients(0, 0, 0),
                hPIDCoeff = new PIDCoefficients(0, 0, 0);


        public static PIDCoefficients
                tPIDCoeff_finalAdj = new PIDCoefficients(0, 0, 0),
                xPIDCoeff_finalAdj = new PIDCoefficients(0, 0, 0),
                yPIDCoeff_finalAdj = new PIDCoefficients(0, 0, 0),
                hPIDCoeff_finalAdj = new PIDCoefficients(0, 0, 0);
    }
    //GoToPoint Constants ↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑


    //Chassis Constants ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
    public static final double xMaxVelocity = 1; //lateral
    public static final double yMaxVelocity = 1; //forward
    public static Vector frontLeftVector = new Vector(-xMaxVelocity, yMaxVelocity).scaleToMagnitude(1);

    //TODO Copy values with voltage correction
    public static double[] minPowersToOvercomeStaticFriction = new double[]{
            0,// leftFront
            0,// leftBack
            0,// rightFront
            0// rightBack
    };
    public static double minPowerToOvercomeKineticFriction = 0;
    public static final int SWITCH_FROM_STATIC_TO_KINETIC_FRICTION = 0;//In MS


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
        public static double TotalMassOfRobot = 1; //(In KG)
        public static double CentripetalScalingFactor = 0;
    }










    public static final double WAIT_TIME_VARIABLE = 99999999.991; //IGNORA
}