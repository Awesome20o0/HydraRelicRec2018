package org.firstinspires.ftc.teamcode.Lernaean.ModuleTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Libraries.Lift;

/**
 * Created by Arib on 2/21/2017.
 */

@Autonomous(name = "Judging", group = "LinearOpMode")
@Disabled
public class Judging extends LinearOpMode {
    private Lift lift;

    @Override
    public void runOpMode() throws InterruptedException {
        lift = new Lift(this);

        lift.armsOut();

        lift.armRelease();

        waitForStart();

        lift.armsGrab();

        Thread.sleep(1000);

        lift.activateLift();

        Thread.sleep(2000);

        lift.unactivateLift();

        while(opModeIsActive());

        lift.armsDrop();

        Thread.sleep(1500);
    }
}