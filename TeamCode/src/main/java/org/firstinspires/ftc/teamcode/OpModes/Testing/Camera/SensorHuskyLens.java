/*
Copyright (c) 2023 FIRST

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.OpModes.Testing.Camera;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.core.Util.Hardware.BetterServo;
import org.firstinspires.ftc.teamcode.core.Util.Hardware.CoolMotor;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;

import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates how to use the DFRobot HuskyLens.
 *
 * The HuskyLens is a Vision Sensor with a built-in object detection model.  It can
 * detect a number of predefined objects and AprilTags in the 36h11 family, can
 * recognize colors, and can be trained to detect custom objects. See this website for
 * documentation: https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336
 *
 * For detailed instructions on how a HuskyLens is used in FTC, please see this tutorial:
 * https://ftc-docs.firstinspires.org/en/latest/devices/huskylens/huskylens.html
 *
 * This sample illustrates how to detect AprilTags, but can be used to detect other types
 * of objects by changing the algorithm. It assumes that the HuskyLens is configured with
 * a name of "huskylens".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@Config
@TeleOp(name = "Sensor: HuskyLens", group = "Sensor")
public class SensorHuskyLens extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime();
    private HuskyLens huskyLens;
    BetterServo clawIntake, pivotClaw, wristClaw_Intake, pivotIntake, armIntakeLeft, armIntakeRight, clawOuttake, wristClaw_Outtake, armOuttake;
    MecanumDrive drive;
    CoolMotor linkage;
    public static double kP = 0.005, kI = 0, kD = 0.00008;
    public static double target = 1200;
    PIDController pid = new PIDController(kP, kI, kD);
    double lastPos = 0.32;
    double angle;
    double lastTarget;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        clawIntake = new BetterServo(hardwareMap.get(Servo.class, "CI"));
        pivotClaw = new BetterServo(hardwareMap.get(Servo.class, "PC"));
        pivotIntake = new BetterServo(hardwareMap.get(Servo.class, "PI"));
        wristClaw_Intake = new BetterServo(hardwareMap.get(Servo.class, "WCI"));
        armIntakeLeft = new BetterServo(hardwareMap.get(Servo.class, "AIL"));
        armIntakeRight = new BetterServo(hardwareMap.get(Servo.class, "AIR"));
        clawOuttake = new BetterServo(hardwareMap.get(Servo.class, "CO"));
        wristClaw_Outtake = new BetterServo(hardwareMap.get(Servo.class, "WCO"));
        armOuttake = new BetterServo(hardwareMap.get(Servo.class, "AO"));
        drive = new MecanumDrive(hardwareMap, telemetry, true);
        linkage = new CoolMotor(hardwareMap.get(DcMotorEx.class, "L"), "linkage");
        linkage.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linkage.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linkage.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*
         * This sample rate limits the reads solely to allow a user time to observe
         * what is happening on the Driver Station telemetry.  Typical applications
         * would not likely rate limit.
         *

        /*
         * Basic check to see if the device is alive and communicating.  This is not
         * technically necessary here as the HuskyLens class does this in its
         * doInitialization() method which is called when the device is pulled out of
         * the hardware map.  However, sometimes it's unclear why a device reports as
         * failing on initialization.  In the case of this device, it's because the
         * call to knock() failed.
         */
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        /*
         * The device uses the concept of an algorithm to determine what types of
         * objects it will look for and/or what mode it is in.  The algorithm may be
         * selected using the scroll wheel on the device, or via software as shown in
         * the call to selectAlgorithm().
         *
         * The SDK itself does not assume that the user wants a particular algorithm on
         * startup, and hence does not set an algorithm.
         *
         * Users, should, in general, explicitly choose the algorithm they want to use
         * within the OpMode by calling selectAlgorithm() and passing it one of the values
         * found in the enumeration HuskyLens.Algorithm.
         *
         * Other algorithm choices for FTC might be: OBJECT_RECOGNITION, COLOR_RECOGNITION or OBJECT_CLASSIFICATION.
         */
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        telemetry.update();
        pivotIntake.setPosition(0.32);
        pivotClaw.setPosition(1);
        waitForStart();

        /*
         * Looking for AprilTags per the call to selectAlgorithm() above.  A handy grid
         * for testing may be found at https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336#target_20.
         *
         * Note again that the device only recognizes the 36h11 family of tags out of the box.
         */
        while (opModeIsActive()) {
            drive.motors.drive(gamepad1);
            drive.update();

            if (gamepad1.a) {
                armIntakeRight.setPosition(0.15);
                armIntakeLeft.setPosition(0.15);
                wristClaw_Intake.setPosition(1);
                pivotIntake.setPosition(0.32);
                lastPos = 0.32;
                pivotClaw.setPosition(1);
            }
            if (gamepad1.b) {
                pid.setSetPoint(0);
                lastTarget = 0;
                armIntakeRight.setPosition(0.4);
                armIntakeLeft.setPosition(0.4);
                wristClaw_Intake.setPosition(0.3);
                pivotIntake.setPosition(0.32);
                lastPos = 0.32;
            }
            if (gamepad1.left_bumper)
                clawIntake.setPosition(1);//deschis
            if (gamepad1.right_bumper)
                clawIntake.setPosition(0.55);//inchis

            /*
             * All algorithms, except for LINE_TRACKING, return a list of Blocks where a
             * Block represents the outline of a recognized object along with its ID number.
             * ID numbers allow you to identify what the device saw.  See the HuskyLens documentation
             * referenced in the header comment above for more information on IDs and how to
             * assign them to objects.
             *
             * Returns an empty array if no objects are seen.
             */
            HuskyLens.Block[] blocks = huskyLens.blocks();
            int r = 0;
            telemetry.addData("Block count", blocks.length);
            for (int i = 0; i < blocks.length; i++) {
                //telemetry.addData("Block", blocks[i].toString());
                if (blocks[i].id != 0) {
                    switch (blocks[i].id) {
                        case 1:
                            telemetry.addLine("Galben" + " Pozitie " + blocks[i].x + "X" + blocks[i].y + "Dimensiune" + blocks[i].width + "x" + blocks[i].height);
                            break;
                        case 2:
                            telemetry.addLine("Rosu" + " Pozitie " + blocks[i].x + "X" + blocks[i].y + "Dimensiune" + blocks[i].width + "x" + blocks[i].height);
                            break;
                        case 3:
                            telemetry.addLine("Albastru" + " Pozitie " + blocks[i].x + "X" + blocks[i].y + "Dimensiune" + blocks[i].width + "x" + blocks[i].height);
                            break;
                    }
                    angle = Math.atan2(blocks[r].height, blocks[r].width);
                    r = i;
                }
                /*
                 * Here inside the FOR loop, you could save or evaluate specific info for the currently recognized Bounding Box:
                 * - blocks[i].width and blocks[i].height   (size of box, in pixels)
                 * - blocks[i].left and blocks[i].top       (edges of box)
                 * - blocks[i].x and blocks[i].y            (center location)
                 * - blocks[i].id                           (Color ID)
                 *
                 * These values have Java type int (integer).
                 */
            }
            if (gamepad1.dpad_up) {
                pid.setSetPoint(target);
                lastTarget = target;
            }
            if (gamepad1.dpad_down) {
                pid.setSetPoint(0);
                lastTarget = 0;
            }

            if (gamepad1.y && timer.milliseconds() >= 300) {
                r = 0;
                double target = blocks[r].x;
                double currentPos = Range.scale(target, 320, 0, lastPos - 0.065, lastPos + 0.065);
                pivotIntake.setPosition(currentPos);

                double slideTarget = lastTarget + Range.scale(blocks[r].y, 240, 0, -700, 700);
                pid.setSetPoint(slideTarget);
                //angle = Math.atan((double) blocks[r].height / blocks[r].width);
                //pivotClaw.setPosition(Range.scale(Math.toDegrees(angle), 25, 110, 1, 0));
                if (blocks[r].width > blocks[r].height && Math.abs(blocks[r].width - blocks[r].height) >= 30) {
                    pivotClaw.setPosition(0.5);
                    telemetry.addData("Poz servo: ", 0.5);
                } else if (blocks[r].width >= blocks[r].height && Math.abs(blocks[r].width - blocks[r].height) <= 30) {
                    pivotClaw.setPosition(0.7);
                    telemetry.addData("Poz servo: ", 0.7);
                } else if (blocks[r].width <= blocks[r].height && Math.abs(blocks[r].width - blocks[r].height) <= 30) {
                    pivotClaw.setPosition(0.35);
                    telemetry.addData("Poz servo: ", 0.35);
                } else {
                    pivotClaw.setPosition(1);
                    telemetry.addData("Poz servo: ", 1);
                }
                lastPos = currentPos;
                lastTarget = slideTarget;
                timer.reset();
            }
            if (gamepad1.x) {
                armIntakeRight.setPosition(0.005);
                armIntakeLeft.setPosition(0.005);
                wristClaw_Intake.setPosition(1);
            }
            clawOuttake.update();
            clawIntake.update();
            pivotClaw.update();
            armIntakeLeft.update();
            armIntakeRight.update();
            wristClaw_Outtake.update();
            wristClaw_Intake.update();
            pid.setPID(kP, kI, kD);
            linkage.setPowerForced(pid.calculate(linkage.motor.getCurrentPosition()));
            linkage.update();
            telemetry.addData("Curr pos", linkage.motor.getCurrentPosition());
            telemetry.addData("Angle", Math.toDegrees(angle));
            telemetry.update();
        }
    }
}