package org.firstinspires.ftc.teamcode.Lernaean;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Libraries.BeaconPushers;
import org.firstinspires.ftc.teamcode.Libraries.Drivetrain;
import org.firstinspires.ftc.teamcode.Libraries.Lift;
import org.firstinspires.ftc.teamcode.Libraries.Manipulator;
import org.firstinspires.ftc.teamcode.Libraries.Sensor;
import org.firstinspires.ftc.teamcode.Libraries.Shooter;

/**
 * Created by Arib on 10/20/2016.
 */
@Autonomous(name = "RedAutonomous", group = "LinearOpMode")
@Disabled
public class RedAutonomous extends LinearOpMode {
    //Create robot objects
    private Drivetrain drivetrain;
    private Manipulator manipulator;
    private Shooter shooter;
    private BeaconPushers beaconPushers;
    private Lift lift;
    private Sensor sensor;

    //create class variables
    private double voltage;
    private String version;

    @Override
    public void runOpMode() throws InterruptedException {

        //initialize the robot
        drivetrain = new Drivetrain(this);
        manipulator = new Manipulator(this);
        shooter = new Shooter(this);
        beaconPushers = new BeaconPushers(this);
        lift = new Lift(this);

        //create telemetry data
        composeTelemetry();

        //wait two seconds to regain voltage drop from init
        Thread.sleep(2000);

        //calculate the voltage
        voltage = hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage();

        /* This is the version number of the current iteration
        this is because sometimes the compiling process build the app but then installs
        the old version instead of applying updates. This version numbers is displayed over
        telemetry to ensure the autonomous is running the current version.
         */
        version = "1.136";

        //display the voltage and version for testing
        telemetry.addData("version: ", version);
        telemetry.addData("voltage", voltage);
        telemetry.addData("init", "init fully finished");
        telemetry.update();

        int ballsToShoot = 2;
        boolean parkCenter = true;
        //wait for the program to actually start and display data in the meantime

        //wait for autonmous to actually start in the meantime return values
        while(!opModeIsActive()) {
            if(gamepad1.dpad_up) {
                ballsToShoot++;
                while(gamepad1.dpad_up);
            }
            if(gamepad1.dpad_down) {
                ballsToShoot--;
                while(gamepad1.dpad_down);
            }
            if(gamepad1.a) {
                parkCenter = !parkCenter;
                while (gamepad1.a);
            }
            telemetry.addData("Instructions", "Use D-Pad to change shooting");
            telemetry.addData("Instructions", "Press A to change parking");

            telemetry.addData("Balls to shoot", ballsToShoot);

            if(parkCenter)
                telemetry.addData("Parking", "Center and Cap Ball");
            else
                telemetry.addData("Parking", "Corner Ramp");

            telemetry.update();
            idle();
        }

        //calculate the voltage
        voltage = hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage();

        Thread.sleep(100);

        //start the acceleration calculation for the gyro
        drivetrain.sensor.gyro.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //display that we are moving off the wall
        telemetry.addData("currentStep", "moving off the wall");
        telemetry.update();

        if(ballsToShoot > 0) {
            //move forward to get within shooting range
            drivetrain.moveBackward(.35, 1667, 5000);

            //display that we are going to shoot
            telemetry.addData("currentStep", "shooting");
            telemetry.update();

            //turn the safe off
            manipulator.activateShooter();

            //start the shooter at the calculated power from the voltage value saved
            shooter.startShooter(-shooter.getNeededPower(voltage));

            //wait one second for the shooter to spin-up
            Thread.sleep(1500);

            //start moving the collector
            manipulator.runCollector(-1);

            //let the balls move for 1 second
            Thread.sleep(750);

            //stop the balls from moving
            manipulator.runCollector(0);

            if (ballsToShoot > 1) {
                //wait 1/2 second to wait for spinup
                Thread.sleep(500);

                //let the balls move again
                manipulator.runCollector(-1);

                //keep the balls moving for 1.5 seconds
                Thread.sleep(1000);

                //stop the collector
                manipulator.runCollector(0);
            }

            //stop the shooter
            shooter.stopShooter();


            //move away from the shooting zone
            drivetrain.moveBackward(-.3, 833, 5000);
        } else {
            drivetrain.moveForward(.3, 833, 5000);
        }

        //rotate 40 degrees to the left
        drivetrain.rotateP(.42, -40);

        //stop after the rotation safety stop
        drivetrain.stopMotors();

        //wait 1/4 seconds for momentum
        Thread.sleep(250);

        //display that we are going to move forward
        telemetry.addData("currentStep", "movingForward");
        telemetry.addData("currentAngle", drivetrain.sensor.getGyroYaw());
        telemetry.update();

        //move forward to the wall
        drivetrain.moveForwardToWall(.75, .4, 7350, 10000, 40);

        //move forward into line
        drivetrain.moveFowardToLine(.15, .3, 3000);

        //wait for momentum
        Thread.sleep(250);

        //correct back onto the line
        drivetrain.moveFowardToLine(-.11, -.15, 5000);

        lift.armsDrop();

        //Press the beacon 2 times and on the third time correct a bit before the last push
        boolean blue = beaconPushers.isBackBlue();
        boolean attempted = false;
        int count = 0;
        while (beaconPushers.isBeaconUnpressed()) {
            if(count == 2) {
                if(blue) {
                    drivetrain.moveForward(.11, .15, 100, 500);
                } else {
                    drivetrain.moveForward(-.11, -.15, 100, 500);
                }
            }

            if (blue) {
                beaconPushers.frontPush();
                attempted = true;
            }
            else {
                beaconPushers.backPush();
                attempted = true;
            }
            if(count == 2)
                break;
            count++;
            Thread.sleep(250);
        }

        //if the loop did not attempt at all
        if(!attempted) {
            if (blue) {
                beaconPushers.frontPush();
            }
            else {
                beaconPushers.backPush();
            }
        }

        lift.armsIn();

        //display we are moving forwards
        telemetry.addData("currentStep", "movingForward");
        telemetry.addData("currentAngle", drivetrain.sensor.getGyroYaw());
        telemetry.update();

        //move forward at high speed towards the next beacon
        drivetrain.moveForward(-.25, -.5, 3500, 4500);

        //slow down while finding the line
        boolean failed = drivetrain.moveFowardToLine(-.13, -.2, 3000);

        if(failed) {
            Thread.sleep(250);
            drivetrain.moveFowardToLine(.11, .13, 3000);
        }
        lift.armsDrop();
        //Press the beacon 2 times and on the third time correct a bit before the last push
        count = 0;
        attempted = false;
        blue = beaconPushers.isBackBlue();
        while (beaconPushers.isBeaconUnpressed()) {
            if(count == 2) {
                if(blue) {
                    drivetrain.moveForward(.11, .15, 100, 500);
                } else {
                    drivetrain.moveForward(-.11, -.15, 100, 500);
                }
            }
            if (blue){
                beaconPushers.frontPush();
                attempted = true;
            }
            else {
                beaconPushers.backPush();
                attempted = true;
            }
            if(count == 2)
                break;
            count++;
            Thread.sleep(250);
        }

        if(!attempted) {
            if (blue){
                beaconPushers.frontPush();
            }
            else {
                beaconPushers.backPush();
            }
        }

        lift.armsIn();

        if(parkCenter) {

            //move forward a bit
            drivetrain.moveForward(.6, 1500, 1000);

            //turn off the wall and onto the cap ball
            try {
                while (opModeIsActive() && drivetrain.sensor.getGyroYaw() < 100) {
                    drivetrain.startMotors(.6, 0);
                }
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            //stop the motors
            drivetrain.stopMotors();

            //move to push capball off and push
            drivetrain.moveForward(.8, 1, 4000, 5000);

            //turn to make sure we knock off cap ball
            drivetrain.moveForward(1, 0, 450, 2000);
        } else {
            drivetrain.basicArc(1, 0, 150);
            drivetrain.moveBackward(.5, 10000, 5000);
        }

        //saftey stop for end of program
        drivetrain.stopMotors();
    }

    private void composeTelemetry() {
        telemetry.addLine()
                .addData("Avg", new Func<String>() {
                    @Override public String value() {
                        return "avg: " + drivetrain.getEncoderAvg();
                    }
                });
        telemetry.addLine()
                .addData("ods", new Func<String>() {
                    @Override public String value() {
                        return "ods: " + drivetrain.sensor.leftODS() + " " + drivetrain.sensor.rightODS();
                    }
                });
        telemetry.addLine()
                .addData("gyro", new Func<String>() {
                    @Override public String value() {
                        return "gyro: " + drivetrain.sensor.getGyroYaw();
                    }
                });
        telemetry.addLine()
                .addData("motorLPower", new Func<String>() {
                    @Override public String value() {
                        return "leftPower: " + drivetrain.motorBL.getPower();
                    }
                });
        telemetry.addLine()
                .addData("motorRPower", new Func<String>() {
                    @Override public String value() {
                        return "rightPower: " + drivetrain.motorBR.getPower();
                    }
                });
        telemetry.addLine()
                .addData("Color", new Func<String>() {
                    @Override public String value() {
                        return "Color: " + beaconPushers.getColorVal();
                    }
                });
        telemetry.addLine()
                .addData("BL", new Func<String>() {
                    @Override public String value() {
                        return "BL: " + drivetrain.motorBL.getCurrentPosition();
                    }
                });
        telemetry.addLine()
                .addData("BR", new Func<String>() {
                    @Override public String value() {
                        return "BR: " + drivetrain.motorBR.getCurrentPosition();
                    }
                });
        telemetry.addLine()
                .addData("FL", new Func<String>() {
                    @Override public String value() {
                        return "FL: " + drivetrain.motorFL.getCurrentPosition();
                    }
                });
        telemetry.addLine()
                .addData("FR", new Func<String>() {
                    @Override public String value() {
                        return "FR: " + drivetrain.motorFR.getCurrentPosition();
                    }
                });
    }
}
