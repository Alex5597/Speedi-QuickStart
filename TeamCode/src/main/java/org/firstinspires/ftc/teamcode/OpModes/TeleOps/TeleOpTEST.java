package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.Modules.OutTake_Intake.Arm;
import org.firstinspires.ftc.teamcode.core.Modules.OutTake_Intake.Bucket;
import org.firstinspires.ftc.teamcode.core.Modules.OutTake_Intake.IntakeActive;
import org.firstinspires.ftc.teamcode.core.Modules.OutTake_Intake.LinearSlides;

@TeleOp
public class TeleOpTEST extends LinearOpMode {

    IntakeActive intakeSample;
    Arm arm;
    LinearSlides slides;
    Bucket bucket;
    Servo wrist;

    @Override
    public void runOpMode() throws InterruptedException {
        arm = new Arm(hardwareMap);
        intakeSample = new IntakeActive(hardwareMap);
        slides = new LinearSlides(hardwareMap);
        bucket = new Bucket(hardwareMap);
        wrist = hardwareMap.get(Servo.class, "W");
        wrist.setPosition(0.0);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                slides.setState(LinearSlides.States.Extended);
                intakeSample.setState(IntakeActive.States.Collect);
                sleep(500);
                wrist.setPosition(0.87);
            }
            if (gamepad1.b) {
                slides.setState(LinearSlides.States.Retracted);
                intakeSample.setState(IntakeActive.States.Wait);
                wrist.setPosition(0);
            }
            if (gamepad1.y) {
                intakeSample.setState(IntakeActive.States.Score);
            }
            if (gamepad1.right_bumper) {
                bucket.setState(Bucket.States.Hold);
            }
            if (gamepad1.left_bumper) {
                bucket.setState(Bucket.States.Score);
            }
            telemetry.addData("Sensor read", bucket.getDistanceFromSampleInBucket());
            telemetry.update();
            slides.update();
            arm.update();
            intakeSample.update();
            bucket.update();
        }

    }
}
