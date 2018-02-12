package org.firstinspires.ftc.teamcode.PapaSmurf;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

/**
 * Created by Varun on 9/20/2017.
 */

@Autonomous(name = "REVColorTest", group = "Testing")
@Disabled
public class Color extends LinearOpMode {

    public ColorSensor colorSensorLeft;
    OpMode opMode;

    String version;


    @Override
    public void runOpMode() throws InterruptedException {

        colorSensorLeft = opMode.hardwareMap.colorSensor.get("color");
        colorSensorLeft.setI2cAddress(I2cAddr.create8bit(0x0c));
        colorSensorLeft.enableLed(false);


        version = "1.0";

        telemetry.addData("version", version);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Red: ", colorSensorLeft.red());
            telemetry.addData("Blue: ", colorSensorLeft.blue());
            telemetry.addData("Alpha: ", colorSensorLeft.alpha());
            telemetry.addData("Green: ", colorSensorLeft.green());
            telemetry.update();
        }
    }
}


