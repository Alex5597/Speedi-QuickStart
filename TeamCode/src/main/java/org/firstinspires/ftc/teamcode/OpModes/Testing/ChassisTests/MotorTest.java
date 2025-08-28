package org.firstinspires.ftc.teamcode.OpModes.Testing.ChassisTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MotorTest extends LinearOpMode {

    DcMotorEx Motor1,Motor2,Motor3,Motor4;

    @Override
    public void runOpMode() throws InterruptedException {
        Motor1 = hardwareMap.get(DcMotorEx.class,"M1");
        Motor2 = hardwareMap.get(DcMotorEx.class,"M2");
        Motor3 = hardwareMap.get(DcMotorEx.class,"M3");
        Motor4 = hardwareMap.get(DcMotorEx.class,"M4");

        waitForStart();

        while(opModeIsActive()){
            Motor1.setPower(0);
            Motor2.setPower(0);
            Motor3.setPower(0);
            Motor4.setPower(0);
            if(gamepad1.a){
                Motor1.setPower(1);
            }
            if(gamepad1.b){
                Motor2.setPower(1);
            }
            if(gamepad1.x){
                Motor3.setPower(1);
            }
            if(gamepad1.y){
                Motor4.setPower(1);
            }

            telemetry.addData("Motor 1(a de pe xbox): ", Motor1.getPower());
            telemetry.addData("Motor 2(b de pe xbox): ", Motor2.getPower());
            telemetry.addData("Motor 3(x de pe xbox): ", Motor3.getPower());
            telemetry.addData("Motor 4(y de pe xbox): ", Motor4.getPower());
        }
    }
}
