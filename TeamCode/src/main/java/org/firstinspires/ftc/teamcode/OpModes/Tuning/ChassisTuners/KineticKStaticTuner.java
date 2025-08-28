package org.firstinspires.ftc.teamcode.OpModes.Tuning.ChassisTuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.core.Util.Math.Vector;

@TeleOp(name = "Kinetic Kstatic Tuner")
@Config
public class KineticKStaticTuner extends LinearOpMode {
    public static double scalar = 0.0001;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        MecanumDrive robot = new MecanumDrive(hardwareMap, telemetry, true);
        robot.setRunMode(MecanumDrive.RunMode.MANUAL);
        waitForStart();
        double power = 0.0;

        while (opModeIsActive()) {
            double vely = robot.getLocalizerInstance().getVelocity().getY();

            power += scalar * (vely > 20.0 ? -1 : 1);

            robot.motors.setMotorPowerForced(new Vector(0, power, 0));
            robot.motors.update();
            telemetry.addData("LOOK AT ME", power + "");
            telemetry.addData("Current Velocity copy value from LOOK AT ME when this stays approximately constant", vely);
            telemetry.update();
            robot.update();
        }
    }
}