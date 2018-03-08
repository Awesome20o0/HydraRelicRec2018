package org.firstinspires.ftc.teamcode.PapaSmurf;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Locale;


/**
 * Created by Varun on 9/2/2017.
 */

public abstract class PapaSmurfOpMode extends OpMode {
    // Define all motors, servos, sensors, etc.
    DcMotor motorFL;
    DcMotor motorBL;
    DcMotor motorFR;
    DcMotor motorBR;

    DcMotor intakeL;
    DcMotor intakeR;
    CRServo omnipL;
    CRServo omnipR;

    DcMotor lift;
    DcMotor relic;
    Servo gate;

    Servo pusherR;
    Servo pusherL;

    CRServo outputR;
    CRServo outputL;

    PapaSmurfOpMode opMode;

    Servo hour;
    Servo minute;
    Servo second;

    Servo shoulder1;
    Servo shoulder2;
    Servo wrist;
    Servo hand;
//
//    Servo hookL;
//    Servo hookR;

    public boolean endGame = false;

    // To monitor current voltage
    double voltage = 0.0;


    //For reversing which side is "forward" during the match
//    boolean reversed = true;

    //To be accessed by the actual opMode
    double slowingFactor = 1;
    double powerL;
    double powerR;

    ColorSensor jewelSensorLeft;
    ColorSensor jewelSensorRight;

// Hardware map and update telemetry
    @Override
    public void init() {

        opMode = this;

        int slowFactor = 1;

        motorBL = hardwareMap.dcMotor.get("BL");
        motorBR = hardwareMap.dcMotor.get("BR");
        motorFR = hardwareMap.dcMotor.get("FR");
        motorFL = hardwareMap.dcMotor.get("FL");

        omnipL = hardwareMap.crservo.get("omnipL");
        omnipR = hardwareMap.crservo.get("omnipR");
        intakeL = hardwareMap.dcMotor.get("intakeL");
        intakeR = hardwareMap.dcMotor.get("intakeR");

        outputL = hardwareMap.crservo.get("outputL");
        outputR = hardwareMap.crservo.get("outputR");

        lift = hardwareMap.dcMotor.get("lift");
        relic = hardwareMap.dcMotor.get("relic");

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        relic.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gate = hardwareMap.servo.get("gate");
        hour = hardwareMap.servo.get("hour");

        second = hardwareMap.servo.get("second");
        minute = hardwareMap.servo.get("minute");

//        hookL = hardwareMap.servo.get("hookL");
//        hookR = hardwareMap.servo.get("hookR");

        shoulder1 = hardwareMap.servo.get("shoulder1");
//        shoulder2 = hardwareMap.servo.get("shoulder2");
////        wrist = hardwareMap.servo.get("wrist");
        hand = hardwareMap.servo.get("hand");


//        pusherR = hardwareMap.servo.get("pushR");
//        pusherL = hardwareMap.servo.get("pushL");

        // jewelSensorLeft = opMode.hardwareMap.colorSensor.get("jewelSensorL");
        // jewelSensorRight = opMode.hardwareMap.colorSensor.get("jewelSensorR");

        //REV Expansion Hub Gyro Code
        composeTelemetry();

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.update();

    }


//    public void balance() throws InterruptedException {
//
//        double pitch = getGyroPitch();
//        double roll = getGyroRoll();
//
//        //Push downwards on balancing stone
//        pushersOut();
//        Thread.sleep(500);
//
//        // Move forward
//        startMotors(-.5, -.5);
//        Thread.sleep(1000);
//        pushersIn();
//
//        int quadrant = 1;
//
//        if(roll > 0 && pitch > 0) {
//
//            quadrant = 1;
//        }
//
//        if(roll < 0 && pitch > 0) {
//
//            quadrant = 2;
//        }
//
//        if(roll < 0 && pitch < 0) {
//
//            quadrant = 3;
//        }
//
//        if(roll > 0 && pitch < 0) {
//
//            quadrant = 4;
//        }
//
//        if (Math.abs(roll) < 3){
//            if (pitch > 0){
//                while (Math.abs(pitch) > 3){
//                    startMotors(-.15, -.15);
//                }
//            }else {
//                while (Math.abs(pitch) > 3){
//                    startMotors(.15, .15);
//                }
//            }
//        }
//
//        if (Math.abs(pitch) < 3){
//            if (roll > 0){
//                while (Math.abs(roll) > 3){
//                    move(.15, 0, Math.PI/2);
//                }
//            }else {
//                while (Math.abs(roll) > 3){
//                    move(.15, 0, 0);
//                }
//            }
//        }
//        while((Math.abs(pitch) > 7) && (Math.abs(roll)  > 7)) {
//
//            double tangent = Math.atan(roll/pitch);
//
//            if(quadrant == 1) {
//                move(.15, 0, (tangent - Math.PI));
//
//            } else if (quadrant == 2){
//                move(.15, 0, tangent);
//
//            } else if (quadrant == 3){
//                move(.15, 0, tangent);
//
//            } else if (quadrant == 4){
//                move(.15, 0, (tangent - Math.PI));
//            }
//
//        }
//
//    }

//    public void hourUp() {
//        hour.setPosition(-1);
//    }

    public void hourAndGate() throws InterruptedException{
        armUp();
        Thread.sleep(100);
        gate.setPosition(0);
    }

    public void armUpToIn() throws InterruptedException{

        // This value is straight down
        second.setPosition(.13);
        Thread.sleep(150);
        minute.setPosition(.5);
        Thread.sleep(150);

        hour.setPosition(.9);
        Thread.sleep(150);

        // This value should be straight up for minute hand, slightly lower than parallel with ground
        minute.setPosition(.02);
        Thread.sleep(150);

    }

    public void minuteUp() {
        minute.setPosition(.5);
    }

//    public void hooksDown() {
//        hookL.setPosition(.8);
//        hookR.setPosition(0);
//    }
//
//    public void hooksUp() {
//        hookL.setPosition(0);
//        hookR.setPosition(.8);
//    }

//    public void balance() throws InterruptedException {
//        startMotors(-1, -1);
//        Thread.sleep(800);
//        hooksDown();
//        Thread.sleep(400);
//        stopMotors();
//
//    }

    public void move(double pow, double rotation, double direction) {

        final double FL = pow * Math.cos(direction - Math.PI/4) + rotation;
        final double FR = pow * Math.sin(direction - Math.PI/4) - rotation;
        final double BL = pow * Math.sin(direction - Math.PI/4) + rotation;
        final double BR = pow * Math.cos(direction - Math.PI/4) - rotation;

        motorFL.setPower(-FL);
        motorBL.setPower(-BL);
        motorBR.setPower(BR);
        motorFR.setPower(FR);
    }

//    public void reverse() {
//        reversed = !reversed;
//    }

    public void startMotors(double r, double l) {
//        if (reversed) {
//            motorFL.setPower(l * slowingFactor);
//            motorBL.setPower(l * slowingFactor);
//            motorFR.setPower(-r * slowingFactor);
//            motorBR.setPower(-r * slowingFactor);
//        } else {
            motorFL.setPower(-l * slowingFactor);
            motorBL.setPower(-l * slowingFactor);
            motorFR.setPower(r * slowingFactor);
            motorBR.setPower(r * slowingFactor);
        }
//    }

    public void startMotorsHalf(double r, double l) {
//        if (reversed) {
//            motorFL.setPower(l * .5 * slowingFactor);
//            motorBL.setPower(l * .5 * slowingFactor);
//            motorFR.setPower(-r * .5 * slowingFactor);
//            motorBR.setPower(-r * .5 * slowingFactor);
//        } else {
            motorFL.setPower(-l * .5 * slowingFactor);
            motorBL.setPower(-l * .5 * slowingFactor);
            motorFR.setPower(r * .5 * slowingFactor);
            motorBR.setPower(r * .5 * slowingFactor);
        }
//    }

    public void stopMotors() {
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);
    }

    // Encoder Stuff
    public int getEncoderAvg() {

        // Throws out any bad encoders in attempt to keep accurate average
        int numZeros = 0;
        if (motorBR.getCurrentPosition() == -1) {
            numZeros++;
        }
        if (motorFR.getCurrentPosition() == -1) {
            numZeros++;
        }
        if (motorBL.getCurrentPosition() == -1) {
            numZeros++;
        }
        if (motorFL.getCurrentPosition() == -1) {
            numZeros++;
        }
        return (Math.abs(motorBR.getCurrentPosition()) + Math.abs(motorBL.getCurrentPosition()) +
                Math.abs(motorFR.getCurrentPosition()) + Math.abs(motorFL.getCurrentPosition())) / (4 - numZeros);
    }

    public int getRightEncoderAvg() {
        return (Math.abs(motorFR.getCurrentPosition()) + Math.abs(motorBR.getCurrentPosition())) / 2;
    }

    public int getLeftEncoderAvg() {
        return (Math.abs(motorFL.getCurrentPosition()) + Math.abs(motorBL.getCurrentPosition())) / 2;
    }
    public void intakeIn(){
        intakeL.setPower(-1);
        intakeR.setPower(1);
    }

    public void intakeOut(){
        intakeL.setPower(1);
        intakeR.setPower(-1);
    }

    public void intakeStop(){
        intakeL.setPower(0);
        intakeR.setPower(0);
    }

    public void intakeSpin(){
        intakeL.setPower(-1);
        intakeR.setPower(-1);
    }

    public void outputOut(double power){
        outputL.setPower(power);
        outputR.setPower(-power);
    }

    public void outputIn(double power){
        outputL.setPower(-power);
        outputR.setPower(power);
    }

    public void stopOutput(){
        outputL.setPower(0);
        outputR.setPower(0);
        //herro
    }

    public void omnipUp() {
        omnipR.setPower(-.85);
        omnipL.setPower(.85);
    }

    public void omnipDown() {
        omnipR.setPower(.85);
        omnipL.setPower(-.85);
    }

    public void omnipStop() {
        omnipR.setPower(0);
        omnipL.setPower(0);
    }

    public void relicOut(double power){
        relic.setPower(power);
    }

    public void relicIn(double power){
        relic.setPower(-power);
    }

    public void relicStop() {
        relic.setPower(0);
    }

    public void shoulderDown(){
        shoulder1.setPosition(.90);
    }

    public void shoulderUp(){
        shoulder1.setPosition(0);
    }

    public void shoulderMid() {
        shoulder1.setPosition(.65);
    }

    public void closeHand(){
        hand.setPosition(-.5);
    }

    public void openHand(){
        hand.setPosition(.5);
    }

    public void liftUp(double power){
        lift.setPower(power);
    }

    public void liftDown(double power){
        lift.setPower(-power);
    }

    public void liftStop() {
        lift.setPower(0);
        relic.setPower(0);
    }


//    public void gate() {
//        gate.setPosition(0);
//    }

    public void armOut() throws InterruptedException {
        hour.setPosition(.67);
        Thread.sleep(500);
        minute.setPosition(1);
    }

    public void armUp() throws InterruptedException {
        minute.setPosition(.4);
        Thread.sleep(100);
        hour.setPosition(.2);
        Thread.sleep(100);
        second.setPosition(.13);
    }

//    public void pushersOut(){
//        pusherR.setPosition(-1);
//        pusherL.setPosition(1);
//    }
//
//    public void pushersIn(){
//        pusherR.setPosition(1);
//        pusherL.setPosition(-1);
//    }


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

//    public double getGyroYaw() {
//        updateValues();
//        double yaw = angles.firstAngle * -1;
//        if(angles.firstAngle < -180)
//            yaw -= 360;
//        return yaw;
//    }
//
//
//    public double getGyroPitch() {
//        updateValues();
//        double pitch = angles.secondAngle;
//        return pitch;
//    }
//
//    public double getGyroRoll(){
//        updateValues();
//        double roll = angles.thirdAngle;
//        return roll;
//    }
//
//    public boolean resetGyro() {
//        return gyro.initialize(parameters);
//    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

//    public double getVoltage() {
//        return hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage();
//    }

    private void composeTelemetry() {

        telemetry.addLine()
                .addData("voltage", new Func<String>() {
                    @Override public String value() {
                        return "voltage: " + voltage;
                    }
                });
        telemetry.addLine()
                .addData("BL", new Func<String>() {
                    @Override public String value() {
                        return "BL: " + motorBL.getCurrentPosition();
                    }
                });
        telemetry.addLine()
                .addData("BR", new Func<String>() {
                    @Override public String value() {
                        return "BR: " + motorBR.getCurrentPosition();
                    }
                });
        telemetry.addLine()
                .addData("FL", new Func<String>() {
                    @Override public String value() {
                        return "FL: " + motorFL.getCurrentPosition();
                    }
                });
        telemetry.addLine()
                .addData("FR", new Func<String>() {
                    @Override public String value() {
                        return "FR: " + motorFR.getCurrentPosition();
                    }
                });
        telemetry.addLine()
                .addData("Mode", new Func<String>() {
                    @Override public String value() {
                        return ":" + endGame;
                    }
                });
    }
}