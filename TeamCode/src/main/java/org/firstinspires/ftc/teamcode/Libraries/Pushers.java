package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by willi on 10/10/2017.
 */

public class Pushers {
    LinearOpMode opMode;
    Servo pusherR;
    Servo pusherL;

    private final String LOG_TAG = "DriveTrain";
    public Pushers(LinearOpMode opMode)throws InterruptedException{
        this.opMode = opMode;
        pusherR = this.opMode.hardwareMap.servo.get("pushR");
        pusherL = this.opMode.hardwareMap.servo.get("pushL");
        this.opMode.telemetry.addData("init", "finished drivetrain init");
        this.opMode.telemetry.update();
        this.opMode.telemetry.addData("init", "init finished");
        this.opMode.telemetry.update();
    }

    public void pushersOut(){
        pusherR.setPosition(-1);
        pusherL.setPosition(1);
    }

    public void pushersIn(){
        pusherR.setPosition(1);
        pusherL.setPosition(-1);
    }
}
