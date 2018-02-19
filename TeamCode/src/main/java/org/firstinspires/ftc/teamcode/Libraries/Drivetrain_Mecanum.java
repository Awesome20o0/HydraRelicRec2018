package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Varun on 9/11/2017.
 */

public class Drivetrain_Mecanum{
    public DcMotor motorBL;
    public DcMotor motorFL;
    public DcMotor motorBR;
    public DcMotor motorFR;

    public SensorRR sensor;
    int nullValue;
    LinearOpMode opMode;
    BNO055IMU imu;

    public Drivetrain_Mecanum(LinearOpMode opMode)throws InterruptedException {
        this.opMode = opMode;
        nullValue = 0;
        motorFL = this.opMode.hardwareMap.dcMotor.get("FL");
        motorFR = this.opMode.hardwareMap.dcMotor.get("FR");
        motorBL = this.opMode.hardwareMap.dcMotor.get("BL");
        motorBR = this.opMode.hardwareMap.dcMotor.get("BR");
        this.opMode.telemetry.addData("init", "finished drivetrain init");
        this.opMode.telemetry.update();
        sensor = new SensorRR(opMode, true);
        this.opMode.telemetry.addData("init", "init finished");
        this.opMode.telemetry.update();
    }

    public void resetEncoders() throws InterruptedException {

        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();

        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();

    }

    public int getEncoderAvg() {
        return ((Math.abs(motorBR.getCurrentPosition())) +
                Math.abs(motorBL.getCurrentPosition()) +
                (Math.abs(motorFR.getCurrentPosition())) +
                Math.abs(motorFL.getCurrentPosition())) / 4;
    }


    public void movepid(double power, int distance, double floor, double kP, double kI, double kD, int accuracy, double rotation, double direction, double timeout) throws InterruptedException {

        double error;
        double inte = 0;
        double der;
        double previousRunTime;

        resetEncoders();

        double previousError = distance - getEncoderAvg();

        ElapsedTime runtime = new ElapsedTime();

        opMode.telemetry.addData("distance left", distance + "");
        opMode.telemetry.addData("current Encoder", getEncoderAvg() + "");
        opMode.telemetry.update();

        runtime.reset();

        while((((getEncoderAvg() < (distance - accuracy)) && runtime.seconds() < timeout)) && opMode.opModeIsActive()) {
            error = Math.abs(distance) - Math.abs(getEncoderAvg());
            previousRunTime = runtime.seconds();
            power = (power * (error) * kP) + floor;
            inte += (((runtime.seconds() - previousRunTime)) * error * kI);
            der = (((error - previousError) / (runtime.seconds() - previousRunTime)) * kD);

            power = power + inte - der;

            Range.clip(power, -1, 1);
            move(-power, rotation, direction);

            opMode.telemetry.addData("error", error);
            opMode.telemetry.addData("PID", power);
            opMode.telemetry.addData("integral", inte);
            opMode.telemetry.addData("Encoder", getEncoderAvg());

            opMode.telemetry.update();
//            opMode.telemetry.addData("integral", inte);
            previousError = error;
            opMode.idle();
        }
        opMode.telemetry.update();
        stopMotors();
    }

    public void movepid(double power, int distance, double floor, double kP, double kI, double kD, int accuracy, double rotation, double direction) throws InterruptedException {

        double error;
        double inte = 0;
        double der;
        double previousRunTime;

        resetEncoders();

        double previousError = distance - getEncoderAvg();

        ElapsedTime runtime = new ElapsedTime();

        opMode.telemetry.addData("distance left", distance + "");
        opMode.telemetry.addData("current Encoder", getEncoderAvg() + "");
        opMode.telemetry.update();

        runtime.reset();

        while((getEncoderAvg() < (distance - accuracy)) && (opMode.opModeIsActive())) {
            error = Math.abs(distance) - Math.abs(getEncoderAvg());
            previousRunTime = runtime.seconds();
            power = (power * (error) * kP) + floor;
            inte += ((runtime.seconds() - previousRunTime) * error * kI);
            der = (error - previousError) / (runtime.seconds() - previousRunTime) * kD;

            power = power + inte - der;

            Range.clip(power, -1, 1);
            move(-power, rotation, direction);

            opMode.telemetry.addData("error", error);
            opMode.telemetry.addData("PID", power);
            opMode.telemetry.addData("integral", inte);
            opMode.telemetry.addData("Encoder", getEncoderAvg());

            opMode.telemetry.update();
//            opMode.telemetry.addData("integral", inte);
            previousError = error;
            opMode.idle();
        }

        opMode.telemetry.update();
        stopMotors();
    }

    public void strafepid(double power, int distance, double floor, double kP, double kI, double kD, int accuracy, double direction) throws InterruptedException {

        double error;
        double inte = 0;
        double der;
        double previousRunTime;

        resetEncoders();

        double previousError = distance - getEncoderAvg();

        ElapsedTime runtime = new ElapsedTime();

        opMode.telemetry.addData("distance left", distance + "");
        opMode.telemetry.addData("current Encoder", getEncoderAvg() + "");
        opMode.telemetry.update();

        runtime.reset();

        while(getEncoderAvg() < (distance - accuracy)) {
            error = Math.abs(distance) - Math.abs(getEncoderAvg());
            previousRunTime = runtime.seconds();
            power = (power * (error) * kP) + floor;
            inte += ((runtime.seconds() - previousRunTime) * error * kI);
            der = (error - previousError) / (runtime.seconds() - previousRunTime) * kD;

            power = power + inte + der;

            Range.clip(power, -1, 1);
            strafe(power, power, direction);

            opMode.telemetry.addData("error", error);
            opMode.telemetry.addData("PID", power);
            opMode.telemetry.addData("integral", inte);
            opMode.telemetry.addData("Encoder", getEncoderAvg());

            opMode.telemetry.update();
//            opMode.telemetry.addData("integral", inte);
            previousError = error;
            opMode.idle();
        }

        opMode.telemetry.update();
        stopMotors();
    }

    public void strafepid(double power, int distance, double floor, double kP, double kI, double kD, int accuracy, double direction, double timeout) throws InterruptedException {

        double error;
        double inte = 0;
        double der;
        double previousRunTime;

        double heading = sensor.getGyroYaw();

        resetEncoders();

        double previousError = distance - getEncoderAvg();

        ElapsedTime runtime = new ElapsedTime();

        opMode.telemetry.addData("distance left", distance + "");
        opMode.telemetry.addData("current Encoder", getEncoderAvg() + "");
        opMode.telemetry.update();

        runtime.reset();

        while((getEncoderAvg() < (distance - accuracy)) && (runtime.seconds() < timeout) && opMode.opModeIsActive()) {
            error = Math.abs(distance) - Math.abs(getEncoderAvg());
            previousRunTime = runtime.seconds();
            power = (power * (error) * kP) + floor;
            inte += ((runtime.seconds() - previousRunTime) * error * kI);
            der = (error - previousError) / (runtime.seconds() - previousRunTime) * kD;

            power = power + inte + der;


            Range.clip(power, -1, 1);

            if (sensor.getGyroYaw() > heading + 2)
                strafe(power, power, direction);
            else if (sensor.getGyroYaw() < heading - 2)
                strafe(power, power, direction);
            else
                strafe(power, power, direction);


            opMode.telemetry.addData("error", error);
            opMode.telemetry.addData("PID", power);
            opMode.telemetry.addData("integral", inte);
            opMode.telemetry.addData("Encoder", getEncoderAvg());

            opMode.telemetry.update();
//            opMode.telemetry.addData("integral", inte);
            previousError = error;
            opMode.idle();
        }

        opMode.telemetry.update();
        stopMotors();
    }

    public void move(double pow, double rotation, double direction) {

        final double FL = pow * Math.cos(direction - Math.PI/4) + rotation;
        final double FR = pow * Math.sin(direction - Math.PI/4) - rotation;
        final double BL = pow * Math.sin(direction - Math.PI/4) + rotation;
        final double BR = pow * Math.cos(direction - Math.PI/4) - rotation;

        motorFL.setPower(-FL);
        motorBL.setPower(-BL);
        motorBR.setPower(BR);
        motorFR.setPower(FR);

//    public void move(double pow, double rotation, double direction) {
//
//        final double FL = pow * Math.sin(direction - Math.PI/4) + rotation;
//        final double FR = pow * Math.cos(direction - Math.PI/4) - rotation;
//        final double BL = pow * Math.sin(direction - Math.PI/4) + rotation;
//        final double BR = pow * Math.cos(direction - Math.PI/4) - rotation;
//
//        motorFL.setPower(FL);
//        motorBL.setPower(BL);
//        motorBR.setPower(BR);
//        motorFR.setPower(FR);
    }

    public void strafe(double diag1, double diag2, double direction, double current){

        // Passing positive numbers

        // Negatives
        final double FL = diag1 * Math.sin(direction - Math.PI/4);
        final double FR = diag2 * Math.sin(direction - Math.PI/4);

        // Positives
        final double BL = diag2 * Math.cos(direction - Math.PI/4);
        final double BR = diag1 * Math.cos(direction - Math.PI/4);

        if(sensor.getGyroYaw() < current)
        {
            // Make this number bigger
            motorFL.setPower(FL - ((current - sensor.getGyroYaw()) * .01));
            motorBL.setPower(BL);

            // Make this number smaller
            motorBR.setPower(BR - ((current - sensor.getGyroYaw()) * .01));
            motorFR.setPower(FR);
        }
        else if (sensor.getGyroYaw() > current)
        {
            motorFL.setPower(FL);
            // Make this number bigger
            motorBL.setPower(BL + ((sensor.getGyroYaw() - current) * .01));
            motorBR.setPower(BR);
            //  Make this number smaller
            motorFR.setPower(FR + ((sensor.getGyroYaw() - current) * .01));
        }
        else
        {
            motorFL.setPower(FL);
            motorBL.setPower(BL);
            motorBR.setPower(BR);
            motorFR.setPower(FR);
        }
    }

    public void strafeRed(double diag1, double diag2, double direction, double current){

        // Passing positive numbers

        // Negatives
        final double FL = diag1 * Math.sin(direction - Math.PI/4);
        final double FR = diag2 * Math.sin(direction - Math.PI/4);

        // Positives
        final double BL = diag2 * Math.cos(direction - Math.PI/4);
        final double BR = diag1 * Math.cos(direction - Math.PI/4);

        if(sensor.getGyroYaw() < current)
        {
            // Make this number bigger
            motorFL.setPower(FL - ((current - sensor.getGyroYaw()) * .01));
            motorBL.setPower(BL);

            // Make this number smaller
            motorBR.setPower(BR - ((current - sensor.getGyroYaw()) * .01));
            motorFR.setPower(FR);
        }
        else if (sensor.getGyroYaw() > current)
        {
            motorFL.setPower(FL);
            // Make this number bigger
            motorBL.setPower(BL + ((sensor.getGyroYaw() - current) * .01));
            motorBR.setPower(BR);
            //  Make this number smaller
            motorFR.setPower(FR + ((sensor.getGyroYaw() - current) * .01));
        }
        else
        {
            motorFL.setPower(FL);
            motorBL.setPower(BL);
            motorBR.setPower(BR);
            motorFR.setPower(FR);
        }

    }

    public void strafe(double diag1, double diag2, double direction){

        final double FL = diag1 * Math.sin(direction - Math.PI/4);
        final double FR = diag2 * Math.sin(direction - Math.PI/4);
        final double BL = diag2 * Math.cos(direction - Math.PI/4);
        final double BR = diag1 * Math.cos(direction - Math.PI/4);

        motorFL.setPower(FL);
        motorBL.setPower(BL);
        motorBR.setPower(BR);
        motorFR.setPower(FR);

    }

    public void strafeLeft(double diag1, double diag2, double direction, double current){

        final double FL = diag1 * Math.sin(direction - Math.PI/4);
        final double FR = diag2 * Math.sin(direction - Math.PI/4);
        final double BL = diag2 * Math.cos(direction - Math.PI/4);
        final double BR = diag1 * Math.cos(direction - Math.PI/4);

        if(sensor.getGyroYaw() < current)
        {
            motorFL.setPower(FL);
            motorBL.setPower(BL - ((current - sensor.getGyroYaw()) * .01));
            motorBR.setPower(BR);
            motorFR.setPower(FR - ((current - sensor.getGyroYaw()) * .01));

        } else if (sensor.getGyroYaw() > current)
        {
            motorFL.setPower(FL + ((sensor.getGyroYaw() - current) * .01));
            motorBL.setPower(BL);
            motorBR.setPower(BR + ((sensor.getGyroYaw() - current) * .01));
            motorFR.setPower(FR);
        }
        else {
            motorFL.setPower(FL);
            motorBL.setPower(BL);
            motorBR.setPower(BR);
            motorFR.setPower(FR);
        }
    }

    public void strafeLeftRed(double diag1, double diag2, double direction, double current){

        final double FL = diag1 * Math.sin(direction - Math.PI/4);
        final double FR = diag2 * Math.sin(direction - Math.PI/4);
        final double BL = diag2 * Math.cos(direction - Math.PI/4);
        final double BR = diag1 * Math.cos(direction - Math.PI/4);

        if(sensor.getGyroYaw() < current)
        {
            motorFL.setPower(FL);
            motorBL.setPower(BL - ((current - sensor.getGyroYaw()) * .01));
            motorBR.setPower(BR);
            motorFR.setPower(FR - ((current - sensor.getGyroYaw()) * .01));

        } else if (sensor.getGyroYaw() > current)
        {
            motorFL.setPower(FL + ((sensor.getGyroYaw() - current) * .01));
            motorBL.setPower(BL);
            motorBR.setPower(BR + ((sensor.getGyroYaw() - current) * .01));
            motorFR.setPower(FR);
        }
        else {
            motorFL.setPower(FL);
            motorBL.setPower(BL);
            motorBR.setPower(BR);
            motorFR.setPower(FR);
        }
    }

    public void move(double pow, double rotation, double direction, int encoder) {

        while ((getEncoderAvg() < encoder) && (opMode.opModeIsActive()))
            move(pow, rotation, direction);
    }

    public void startMotors(double ri, double le) throws InterruptedException {
            motorBL.setPower(-le);
            motorFL.setPower(-le);
            motorBR.setPower(ri);
            motorFR.setPower(ri);
        }
    public void stopMotors() throws InterruptedException {
        motorBR.setPower(0);
        motorBL.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);
    }

    public void setNullValue() {
        nullValue = getEncoderAvg();
    }

    public void pid(double power, int angleTo, double floor, double kP, double kI, double kD, int accuracy, double timeout) throws InterruptedException {

        double error;
        double inte = 0;
        double der;

        setNullValue();

        double currentAngle = sensor.getGyroYaw();
        double previousError;
        double startTime;

        ElapsedTime runtime = new ElapsedTime();

        opMode.telemetry.addData("Current Angle", currentAngle + "");
        opMode.telemetry.addData("Angle To", angleTo + "");
        opMode.telemetry.update();

        runtime.reset();

        if(currentAngle < angleTo) {

            previousError = angleTo - currentAngle;

            while(currentAngle < angleTo - accuracy && (runtime.seconds() < timeout) && (opMode.opModeIsActive())) {
                startTime = runtime.seconds();
                currentAngle = sensor.getGyroYaw();

                error = angleTo - currentAngle;

                power = ( power * (error) * kP) + floor;
                inte += (((runtime.seconds() - startTime)) * error * kI);
                der = (((error - previousError) / (runtime.seconds() - startTime)) * kD);

                power = power + inte - der;

                Range.clip(power, -1, 1);
                power = Math.abs(power);
                startMotors(-power, power);

                opMode.telemetry.addData("error", error);
                opMode.telemetry.addData("PID", power);
                opMode.telemetry.addData("integral", inte);
                opMode.telemetry.addData("angle", currentAngle);

                opMode.telemetry.update();
                previousError = error;
                opMode.idle();
            }
            stopMotors();
        }
        else if(currentAngle > angleTo ){

            previousError = currentAngle - angleTo;

            while(currentAngle > angleTo + accuracy  && (runtime.seconds() < timeout) && (opMode.opModeIsActive())) {
                startTime = runtime.seconds();
                currentAngle = sensor.getGyroYaw();

                error = currentAngle - angleTo;

                power = ( power * (error) * kP) + floor;
                inte += (((runtime.seconds() - startTime)) * error * kI);
                der = (((error - previousError) / (runtime.seconds() - startTime)) * kD);

                power = power + inte - der;

                Range.clip(power, -1, 1);
                power = Math.abs(power);
                startMotors(power, -power);

                opMode.telemetry.addData("error", error);
                opMode.telemetry.addData("PID", power);
//            opMode.telemetry.addData("integral", inte);
                opMode.telemetry.addData("integral", inte);
                opMode.telemetry.addData("angle", currentAngle);

                opMode.telemetry.update();
                previousError = error;
                opMode.idle();
            }
            stopMotors();
        }


    }

}

