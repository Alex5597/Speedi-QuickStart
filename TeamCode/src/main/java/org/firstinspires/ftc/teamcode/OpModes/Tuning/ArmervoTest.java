package org.firstinspires.ftc.teamcode.OpModes.Tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.Modules.OutTake_Intake.Arm;
import org.firstinspires.ftc.teamcode.core.Util.Hardware.BetterServo;
import org.firstinspires.ftc.teamcode.core.Util.utils.Constants;

@TeleOp
@Config
public class ArmervoTest extends LinearOpMode {
    Arm arm;
    public static double maxSpeed = 1, deceleration = 15, acceleration = 2;

    @Override
    public void runOpMode() throws InterruptedException {
        arm = new Arm(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.y)
                arm.armServo = new BetterServo(hardwareMap.get(Servo.class, "A"), maxSpeed, acceleration, deceleration, Constants.ArmPoses.armCollectPose);
            if (gamepad1.a)
                arm.setState(Arm.States.Collect);
            if (gamepad1.b)
                arm.setState(Arm.States.Place);
            arm.update();
        }
    }
}