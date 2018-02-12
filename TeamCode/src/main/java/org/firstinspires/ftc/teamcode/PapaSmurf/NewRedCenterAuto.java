package org.firstinspires.ftc.teamcode.PapaSmurf;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
//Varun had a B in English, now he has a 90 so he's still trash. He's gonna lose to Ian.
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Libraries.Drivetrain_Mecanum;
import org.firstinspires.ftc.teamcode.Libraries.GlyphScorer;
import org.firstinspires.ftc.teamcode.Libraries.JewelArm;
import org.firstinspires.ftc.teamcode.Libraries.SensorRR;

/**
 * Created by willi on 11/6/2017.
 */
@Autonomous(name = "New Red Center Auto", group = "LinearOpMode")
@Disabled
public class NewRedCenterAuto extends LinearOpMode {
    private GlyphScorer glyphScorer;
    private Drivetrain_Mecanum drivetrainM;
    private String version;
    private SensorRR sensors;
    private JewelArm arm;
    public static final String TAG = "Vuforia VuMark Test";
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;
    int vu = 1;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        drivetrainM = new Drivetrain_Mecanum(this);
        glyphScorer = new GlyphScorer(this);
        sensors = new SensorRR(this, true);
        arm = new JewelArm(this);

        composeTelemetry();

        // Start the actual process of looping
        waitForStart();

        getVuMark();

        // Move omnipulator up to prevent it from hitting the balancing stone
        glyphScorer.liftUp();

        // Knock off correct jewel ball
        arm.armOut();

        Thread.sleep(500);

        int color = sensors.getColorValue();
        if (color > 0) {
            // Move servo clockwise
            arm.armKick(.27);
        } else {
            // Move servo counter clockwise
            arm.armKick(.65);
        }

        Thread.sleep(500);

        arm.armIn();

        Thread.sleep(800);

        drivetrainM.strafe(-.7, -.7, 0);

        Thread.sleep(1500);

        drivetrainM.stopMotors();

        Thread.sleep(150);

        drivetrainM.pid(.3, 175, .1, .0035, .0015, 0, 2, 10);

//        Thread.sleep(500);

        runtime.reset();

        while((sensors.getDistanceL() > 55) && (runtime.seconds() < 7)){
            telemetry.update();
            drivetrainM.strafeLeftRed(-.5, -.5, 0, 175 );
        }
        while(sensors.getDistanceL() < 62){

            telemetry.update();
            drivetrainM.strafeRed(.8, .8, 0, 175 );
        }

        runtime.reset();

        //vu == 1
        if (true) {

            while((sensors.getDistanceL()) < 119 && (runtime.seconds() < 7)){
                telemetry.update();
                drivetrainM.strafeRed(.5, .5, 0, 175 );
            }
            drivetrainM.stopMotors();
        }
//        if (vu == 2) {
//
//            while((sensors.getDistanceL()) < 99.5 && (runtime.seconds() < 7)){
//                telemetry.update();
//                drivetrainM.strafeRed(-.5, -.5, 0, 175 );
//            }
//            drivetrainM.stopMotors();
//        }
//        if (vu == 3) {
//
//            while((sensors.getDistanceL()) < 87 && (runtime.seconds() < 7)){
//                telemetry.update();
//                drivetrainM.strafeRed(-.5, -.5, 0, 175);
//            }
//            drivetrainM.stopMotors();
//        }
//
//        Thread.sleep(500);
////
////        // Move forward and deposit
//        drivetrainM.startMotors(-.3, -.3);
//
//        Thread.sleep(600);
////
//        glyphScorer.outputOut();
////
//        Thread.sleep(700);
////
////        // Back up to park
//        drivetrainM.startMotors(.3, .3);
//
//        Thread.sleep(500);
//
//        drivetrainM.stopMotors();
//
//        Thread.sleep(200);
//
//        drivetrainM.startMotors(-.5, -.5);
//
//        Thread.sleep(500);
//
//        drivetrainM.stopMotors();
//
//        Thread.sleep(200);
//
//        drivetrainM.startMotors(.25, .25);
//
//        Thread.sleep(250);
//
//        drivetrainM.stopMotors();
    }



    private void composeTelemetry() {
        telemetry.addLine()
                .addData("Avg", new Func<String>() {
                    @Override public String value() {
                        return "avg: " + drivetrainM.getEncoderAvg();
                    }
                });
        telemetry.addLine()
                .addData("gyroYaw", new Func<String>() {
                    @Override public String value() {
                        return "gyro yaw: " + drivetrainM.sensor.getGyroYaw();
                    }
                });
        telemetry.addLine()
                .addData("Color", new Func<String>() {
                    @Override public String value() {
                        return "Color: " + sensors.getColorValue();
                    }
                });
        telemetry.addLine()
                .addData("Time", new Func<String>() {
                    @Override public String value() {
                        return "Time " + runtime.seconds();
                    }
                });
//
//        telemetry.addLine()
//                .addData("gyroPitch", new Func<String>() {
//                    @Override public String value() {
//                        return "gyro pitch: " + drivetrainM.sensor.getGyroPitch();
//                    }
////                });
//        telemetry.addLine()
//                .addData("gyroRoll", new Func<String>() {
//                    @Override public String value() {
//                        return "gyro roll: " + drivetrainM.sensor.getGyroRoll();
//                    }
//                });
        telemetry.addLine()
                .addData("vuMark", new Func<String>() {
                    @Override public String value() {
                        return "VU:" + vu;
                    }
                });
        telemetry.addLine()
                .addData("distanceL", new Func<String>() {
                    @Override public String value() {
                        return "distL:" + sensors.getDistanceL();
                    }
                });

        telemetry.addLine()
                .addData("distanceR", new Func<String>() {
                    @Override public String value() {
                        return "distR:" + sensors.getDistanceR();
                    }
                });
    }

    public void getVuMark() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AYPVi+D/////AAAAGWcdhlX" +
                "rGkdFvb06tBr5+AFjSDfw/YB3Am9Am/B21oh9Jy6CyrZzHhH1A7s" +
                "sJo723Ha+8w0KNhmv38iW3hieiGS3ww/zbK7RgfMDhlAN5Ky/BZ2" +
                "s2NUfKLIt32e9E6O23jOumaRs1Tw6BrIpfi0HnCjUwmkVi/Jd2FXUT" +
                "vWOCPRiJ+Sm7J10sdb4612yzZnx/GpwnFsT9AtKamYqDzHs4CYDXlBJXe" +
                "tnon03SnnZjUxK/8NYbFRRIgKE+N/u3qCwSzus8GJkfwPbxMok9xIWwz" +
                "rDnko2yiKqYb5wZlmZBYI722gR6IOmK8qlGJ+f+stBPQyseR7Q468By8u6" +
                "WcucjveY3gVWh3uGbmzRE0BUTNkV";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        //telemetry.addData(">", "Press Play to start");
        relicTrackables.activate();

        // copy pasta from the ftc ppl
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        ElapsedTime times = new ElapsedTime();
        times.reset();

        while (vuMark == RelicRecoveryVuMark.UNKNOWN && times.seconds() < 2 && opModeIsActive()) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
        }
        telemetry.addData("VuMark ", vuMark);

        if (vuMark == RelicRecoveryVuMark.RIGHT) {
            vu = 3;
        } else if (vuMark == RelicRecoveryVuMark.CENTER) {
            vu = 2;
        } else {
            vu = 1;
        }

    }

}



