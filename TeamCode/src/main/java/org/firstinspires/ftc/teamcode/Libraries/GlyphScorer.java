package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Varun on 9/25/2017.
 */
public class GlyphScorer {
    CRServo omnipL;
    CRServo omnipR;
    DcMotor intakeL;
    DcMotor intakeR;
    CRServo outputL;
    CRServo outputR;
    DcMotor lift;
    Servo gate;
    LinearOpMode opMode;
    DcMotor relic;


    private final String LOG_TAG = "DriveTrain";
    public GlyphScorer (LinearOpMode opMode){
        this.opMode = opMode;
        intakeL = this.opMode.hardwareMap.dcMotor.get("intakeL");
        outputL = this.opMode.hardwareMap.crservo.get("outputL");
        outputR = this.opMode.hardwareMap.crservo.get("outputR");
        intakeR = this.opMode.hardwareMap.dcMotor.get("intakeR");
        lift = this.opMode.hardwareMap.dcMotor.get("lift");
        relic = this.opMode.hardwareMap.dcMotor.get("relic");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        relic.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gate = this.opMode.hardwareMap.servo.get("gate");
        this.opMode.telemetry.addData(LOG_TAG + "init", "finished drivetrain init");
        this.opMode.telemetry.update();
    }

    public void intakeIn(double power){
        intakeL.setPower(-power);
        intakeR.setPower(power);
    }

    public void intakeOut(double power){
        intakeL.setPower(power);
        intakeR.setPower(-power);
    }

    public void intakeStop(){
        intakeL.setPower(0);
        intakeR.setPower(0);
    }

    public void intakeSpin(){
        intakeL.setPower(-1);
        intakeR.setPower(-1);
    }


    public void outputOut(){
        outputL.setPower(.8);
        outputR.setPower(-.8);
    }

    public void outputIn(){
        outputL.setPower(-.85);
        outputR.setPower(.85);
    }

    public void stopOutput(){
        outputL.setPower(0);
        outputR.setPower(0);
    }

//    public void liftUp() throws InterruptedException {
//        lift.setPower(1);
//        Thread.sleep(1000);
//        lift.setPower(0);
//    }

    public void liftUp() throws InterruptedException {
        lift.setPower(1);
        Thread.sleep(350);
        lift.setPower(0);
    }


    public void liftDown() throws InterruptedException {
        lift.setPower(-1);
        Thread.sleep(300);
        lift.setPower(0);
    }

    public void liftStop() {
        lift.setPower(0);
    }

    public void relicOut() throws InterruptedException {
        relic.setPower(1);
        Thread.sleep(2000);
        relic.setPower(0);
    }

    public void relicIn() throws InterruptedException {
        relic.setPower(-1);
        Thread.sleep(2000);
        relic.setPower(0);
    }

    public int getIntakeEncoders() {
        return (Math.abs(intakeL.getCurrentPosition()) +
                Math.abs(intakeR.getCurrentPosition())) / 2;
    }


    public void gate(){
        gate.setPosition(0);
    }
}
