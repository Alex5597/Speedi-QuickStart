package org.firstinspires.ftc.teamcode.OpModes.ChassisTuners;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.core.Util.Hardware.Encoder;

@TeleOp
public class cmPerTickCalculus extends LinearOpMode {
    Encoder par, perp;

    @Override
    public void runOpMode() throws InterruptedException {
        par = new Encoder(hardwareMap.get(DcMotorEx.class, "EN_PAR"));
        perp = new Encoder(hardwareMap.get(DcMotorEx.class, "RBM"));

        par.setDirection(Encoder.Direction.REVERSE);
        perp.setDirection(Encoder.Direction.FORWARD);
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("Push the robot forward and the parallel encoder should tick up");
            telemetry.addLine("Push the robot sideways to the right and the perpendicular encoder should tick up");
            telemetry.addLine("If the ticks go down REVERSE in code, download and try again\n\n");


            telemetry.addData("ticks on parallel encoder", par.getCurrentPosition());
            telemetry.addData("ticks on perpendicular encoder", perp.getCurrentPosition());
            telemetry.update();
        }
    }
}
