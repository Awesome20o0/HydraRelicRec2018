package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

/**
 * Created by Varun on 09/25/2017.
 */
public class SensorRR {

    public BNO055IMU gyro;
    LinearOpMode opMode;
    Orientation angles;
    Acceleration gravity;
    BNO055IMU.Parameters parameters;
    ColorSensor jewelSensorLeft;
    ColorSensor jewelSensorRight;
    ModernRoboticsI2cRangeSensor distanceSensorL;
    ModernRoboticsI2cRangeSensor distanceSensorR;

    public SensorRR(LinearOpMode opMode, boolean imuEnabled) throws InterruptedException {

        this.opMode = opMode;
        jewelSensorLeft = opMode.hardwareMap.colorSensor.get("jewelSensorL");
        jewelSensorRight = opMode.hardwareMap.colorSensor.get("jewelSensorR");
        distanceSensorL = opMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "distanceL");
        distanceSensorR = opMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "distanceR");

        if (imuEnabled) {

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            gyro = opMode.hardwareMap.get(BNO055IMU.class, "imu");
            gyro.initialize(parameters);
        }

    }

    public double getDistanceL(){
        double dist = distanceSensorL.getDistance(DistanceUnit.CM);
        while (dist > 1000 || Double.isNaN(dist))
            dist = distanceSensorL.getDistance(DistanceUnit.CM);
        return dist;
    }

    // Change to distance sensor R
    public double getDistanceR(){
//        double dist
//        while (dist > 1000 || Double.isNaN(dist))
//            dist = distanceSensorR.getDistance(DistanceUnit.CM);
        return distanceSensorR.getDistance(DistanceUnit.CM);
    }


    public int getColorValue(){
        double colorVal = 0;

        double blueLeft = jewelSensorLeft.blue();
        double blueRight = jewelSensorRight.blue();
        double redLeft = jewelSensorLeft.red();
        double redRight = jewelSensorRight.red();

        colorVal += blueRight - blueLeft;
        colorVal += redLeft - redRight;

        return (int) colorVal;

    }

    public int getBlue() {
        return jewelSensorLeft.blue();
    }

    public int getRed(){
        return jewelSensorLeft.red();
    }

    public double getGyroYaw() {
        updateValues();
        double yaw = angles.firstAngle * -1;
        if(angles.firstAngle < -180)
            yaw -= 360;
        return yaw;
    }

    public double getGyroPitch() {
        updateValues();
        double pitch = angles.secondAngle;
        return pitch;
    }

    public double getGyroRoll(){
        updateValues();
        double roll = angles.thirdAngle;
        return roll;
    }

    public boolean resetGyro() {
        return gyro.initialize(parameters);
    }

    public void updateValues() {
        angles = gyro.getAngularOrientation();
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
