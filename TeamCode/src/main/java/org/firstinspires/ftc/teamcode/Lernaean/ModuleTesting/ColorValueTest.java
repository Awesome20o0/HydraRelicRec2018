package org.firstinspires.ftc.teamcode.Lernaean.ModuleTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Libraries.BeaconPushers;
import org.firstinspires.ftc.teamcode.Libraries.Drivetrain;
import org.firstinspires.ftc.teamcode.Libraries.Manipulator;
import org.firstinspires.ftc.teamcode.Libraries.Shooter;

/**
 * Created by Arib on 10/31/2016.
 */
@Autonomous(name = "colortest", group = "Testing")
@Disabled
public class ColorValueTest extends LinearOpMode {

    Drivetrain drivetrain;
    Shooter shooter;
    BeaconPushers beaconPushers;
    Manipulator manipulator;

    String version;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new Drivetrain(this);
        shooter = new Shooter(this);
        beaconPushers = new BeaconPushers(this);
        manipulator = new Manipulator(this);

        version = "1.0";

        telemetry.addData("version", version);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Colors", beaconPushers.getColorVal());
            telemetry.update();
        }
    }
}
