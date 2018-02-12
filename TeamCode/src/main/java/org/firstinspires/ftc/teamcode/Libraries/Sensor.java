package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by Arib on 10/10/2016.
 */
public class Sensor {

    public BNO055IMU gyro;
    OpticalDistanceSensor left;
    OpticalDistanceSensor right;
    LinearOpMode opMode;
    Orientation angles;
    Acceleration gravity;
    BNO055IMU.Parameters parameters;
    ColorSensor jewelSensorLeft;
    ColorSensor jewelSensorRight;

    public Sensor(LinearOpMode opMode) throws InterruptedException {
        this.opMode = opMode;
        jewelSensorLeft = opMode.hardwareMap.colorSensor.get("jewelSensorL");
        jewelSensorRight = opMode.hardwareMap.colorSensor.get("jewelSensorR");
        right = opMode.hardwareMap.opticalDistanceSensor.get("odsR");
        left = opMode.hardwareMap.opticalDistanceSensor.get("odsL");

        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        gyro = opMode.hardwareMap.get(BNO055IMU.class, "gyro");
        gyro.initialize(parameters);

//        File file = AppUtil.getInstance().getSettingsFile("AdafruitIMUCalibration.json");
//        String fileString = ReadWriteFile.readFile(file);
//
//        BNO055IMU.CalibrationData calibrationData = BNO055IMU.CalibrationData.deserialize(fileString);
//
//        gyro.writeCalibrationData(calibrationData);

        angles   = gyro.getAngularOrientation();
        gravity  = gyro.getGravity();




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
        double value = angles.firstAngle * -1;
        if(angles.firstAngle < -180)
            value -= 360;
        return value;
    }


    public double getGyroPitch() {
        updateValues();
        return (angles.thirdAngle);
    }

    public boolean isRightLine() throws InterruptedException {
        return right.getRawLightDetected() > 2;
    }

    public boolean isLeftLine() throws InterruptedException {
        return left.getRawLightDetected() > 1;
    }

    public double rightODS() {
        return right.getRawLightDetected();
    }

    public double leftODS() {
        return left.getRawLightDetected();
    }

    public boolean resetGyro() {
        return gyro.initialize(parameters);
    }

    public void updateValues() {
        angles = gyro.getAngularOrientation();
    }

}
