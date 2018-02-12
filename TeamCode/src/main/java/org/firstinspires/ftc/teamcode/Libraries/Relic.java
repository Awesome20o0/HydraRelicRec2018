package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Varun on 11/18/2017.
 */

public class Relic {

    public DcMotor relic;
    LinearOpMode opMode;
    Servo shoulder1;
    Servo hand;

    public Relic (LinearOpMode opMode) throws InterruptedException {
        this.opMode = opMode;
        relic = opMode.hardwareMap.dcMotor.get("relic");
        shoulder1 = opMode.hardwareMap.servo.get("shoulder1");
        hand = opMode.hardwareMap.servo.get("hand");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

}

    public void shoulderDown(){
        shoulder1.setPosition(-1);
    }

    public void shoulderUp(){
        shoulder1.setPosition(1);
    }

    public void shoulderMid() {
        shoulder1.setPosition(0);
    }

    public void closeHand(){
        hand.setPosition(-1);
    }

    public void openHand(){
        hand.setPosition(1);
    }

}