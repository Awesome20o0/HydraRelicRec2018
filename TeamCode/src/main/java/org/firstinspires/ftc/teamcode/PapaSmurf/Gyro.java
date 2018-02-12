package org.firstinspires.ftc.teamcode.PapaSmurf;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Libraries.SensorRR;

/**
 * Created by Avi on 9/30/2017.
 */

@Autonomous(name = "GyroTest", group = "Testing")
@Disabled
public class Gyro extends LinearOpMode {

    public SensorRR gyro;
    String version;


    @Override
    public void runOpMode() throws InterruptedException {

        version = "1.0";

        telemetry.addData("version", version);
        telemetry.update();

        waitForStart();

        gyro.updateValues();

        while (opModeIsActive()) {

            telemetry.addData("Yaw: ", gyro.getGyroYaw());
            telemetry.addData("Roll: ", gyro.getGyroRoll());
            telemetry.addData("Pitch: ", gyro.getGyroPitch());
            telemetry.update();
        }
    }
}

