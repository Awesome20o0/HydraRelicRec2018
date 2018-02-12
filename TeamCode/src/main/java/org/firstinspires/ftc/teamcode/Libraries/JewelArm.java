package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Varun on 9/25/2017.
 */

public class JewelArm {
    private final LinearOpMode opMode;
    Servo hour;
    Servo minute;
    Servo second;

    public SensorRR jewelColor;

    public JewelArm(LinearOpMode opMode)throws InterruptedException {

        this.opMode = opMode;
        hour = this.opMode.hardwareMap.servo.get("hour");
        minute = this.opMode.hardwareMap.servo.get("minute");
        second = this.opMode.hardwareMap.servo.get("second");
        this.opMode.telemetry.addData("init", "finished drivetrain init");
        this.opMode.telemetry.update();
        this.opMode.telemetry.addData("init", "init finished");
        this.opMode.telemetry.update();
        armInit();
    }



    public void armOut() throws InterruptedException {

        // This should
        hour.setPosition(.85);
        minute.setPosition(.5);
        Thread.sleep(400);
    }

    public void armKick(double position) throws InterruptedException {
        second.setPosition(position);
        Thread.sleep(400);

        // Replace this with second value from ArmIn()
//        second.setPosition(.53);
    }

    public void armIn() throws InterruptedException {

        // This value is straight down
        hour.setPosition(.9);
        Thread.sleep(200);

        // This value should be straight up for minute hand, slightly lower than parallel with ground
        minute.setPosition(.02);
        Thread.sleep(200);
    }

    public void armInit() throws InterruptedException {

        // This value is straight down
        hour.setPosition(.9);
        Thread.sleep(200);

        // This value should be straight up for minute hand, slightly lower than parallel with ground
        minute.setPosition(.02);
        Thread.sleep(200);
        second.setPosition(.53);
    }
}
