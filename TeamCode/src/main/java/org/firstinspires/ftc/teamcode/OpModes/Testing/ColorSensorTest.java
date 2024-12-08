package org.firstinspires.ftc.teamcode.OpModes.Testing;

import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.ColorValues.BlueValues;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.ColorValues.RedValues;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.ColorValues.TrashHold;
import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.ColorValues.YellowValues;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRColor;
import org.firstinspires.ftc.teamcode.core.Modules.OutTake_Intake.IntakeActive;

@TeleOp
public class ColorSensorTest extends LinearOpMode {

    ColorSensor sensor;

    IntakeActive.Color color = IntakeActive.Color.None;

    @Override
    public void runOpMode() throws InterruptedException {
        sensor = hardwareMap.get(ColorSensor.class, "IS");
        float hsvValues[] = {0F, 0F, 0F};

        waitForStart();
        while (opModeIsActive()) {
            Color.RGBToHSV(sensor.red() * 8, sensor.green() * 8, sensor.blue() * 8, hsvValues);
            telemetry.addData("GREEN", sensor.green());
            telemetry.addData("BLUE", sensor.blue());
            telemetry.addData("RED", sensor.red());
            telemetry.addData("HUE", hsvValues[0]);
            telemetry.addData("SATURATION", hsvValues[1]);
            telemetry.addData("VALUE", hsvValues[2]);
            telemetry.addData("COLOR", color);
            telemetry.update();
            if (color == IntakeActive.Color.None) {
                if (Math.abs(hsvValues[0] - RedValues[0]) <= TrashHold[0]) {
                    color = IntakeActive.Color.Red;
                } else if (Math.abs(hsvValues[0] - BlueValues[0]) <= TrashHold[0]) {
                    color = IntakeActive.Color.Blue;
                } else if (Math.abs(hsvValues[0] - YellowValues[0]) <= TrashHold[0]) {
                    color = IntakeActive.Color.Yellow;
                } else {
                    color = IntakeActive.Color.None;
                }
            }
            if (gamepad1.x)
            {
                color = IntakeActive.Color.None;
            }
        }
    }
}
