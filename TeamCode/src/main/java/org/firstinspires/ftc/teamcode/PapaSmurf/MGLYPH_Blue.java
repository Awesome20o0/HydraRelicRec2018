package org.firstinspires.ftc.teamcode.PapaSmurf;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
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
 * Created by Varun on 2/19/2018.
 */
@Autonomous(name = "MGlyph_Blue", group = "LinearOpMode")
public class MGLYPH_Blue extends LinearOpMode {
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

        getVuMark();

        // Start the actual process of looping
        waitForStart();

        // Move omnipulator up to prevent it from hitting the balancing stone
        glyphScorer.liftUp();

        // Knock off correct jewel ball
        arm.armOut();

        Thread.sleep(500);

        int color = sensors.getColorValue();
        if (color < 0) {
            // Move servo clockwise
            arm.armKick(.13);
        } else {
            // Move servo counter clockwise
            arm.armKick(.93);
        }

        Thread.sleep(100);

        arm.armInit();

        Thread.sleep(100);

        arm.armUp();

        // Move off of the balancing stone
//        drivetrainM.movepid(.3, 900, .1, .0007, .0004, 0, 25, 0, Math.PI / 2, 5);

        // Move off of the balancing stone
        drivetrainM.movepid(.3, 800, .1, .0007, .0004, 0, 25, 0, Math.PI / 2, 5);

        Thread.sleep(200);

        drivetrainM.pid(.3, -115, .1, .0072, .0036, 0, 2, 3);

        glyphScorer.intakeIn(1);

        runtime.reset();

        while (runtime.seconds() < 1) {
            int currentAvg = glyphScorer.getIntakeEncoders();
            drivetrainM.startMotors(.4, .4);
            Thread.sleep(25);
            if (Math.abs(glyphScorer.getIntakeEncoders() - currentAvg) < 20)
            {
                glyphScorer.intakeIn(-1);
                Thread.sleep(100);
                glyphScorer.intakeSpin();
                Thread.sleep(100);
                glyphScorer.intakeStop();
            }

            glyphScorer.intakeIn(1);
        }

//        drivetrainM.startMotors(.3, .3);
//
//        Thread.sleep(1200);

        drivetrainM.stopMotors();

        Thread.sleep(200);

        runtime.reset();

        drivetrainM.startMotors(-.4, -.4);

        Thread.sleep(350);

        glyphScorer.intakeIn(-1);

        Thread.sleep(100);

        glyphScorer.intakeStop();

        Thread.sleep(350);

        drivetrainM.stopMotors();

        glyphScorer.intakeStop();

        Thread.sleep(200);

        // Turn 90 degrees towards cryptobox
        drivetrainM.pid(.3, 90, .1, .0045, .002, 0, 2, 5);

        if (sensors.getGyroYaw() > 95) {
            while(sensors.getGyroYaw() > 92 && !isStopRequested()){
                drivetrainM.startMotors(.2, -.2);
            }
    }

        Thread.sleep(100);

//        double previous = drivetrainM.getEncoderAvg();
//
//        while (drivetrainM.getEncoderAvg() < 1500 + previous)
//        {
//            drivetrainM.startMotors(-.25, -.25);
//        }
//

        //        Thread.sleep(500);
        while (sensors.getDistanceF() > 55)
        {
            drivetrainM.startMotors(.35, .35);
        }

        drivetrainM.stopMotors();

        Thread.sleep(100);
        // Drive towards cryptobox
//        drivetrainM.movepid(.3, 1750, .1, .00048, .0005, 0, 25, 0, Math.PI / 2, 3.5);

        double power;

        double lowThresh = .35;


        runtime.reset();

        if (vu == 1) {

            drivetrainM.strafe(-.6, -.6, 0, 90);

            Thread.sleep(500);

            while ( Math.abs(sensors.getDistanceL() - 48) > .5 && (runtime.seconds() < 5) && sensors.getDistanceL() != 0 && !isStopRequested())
            {
                double pVal = .028;

                if (Math.abs(sensors.getDistanceL() - 48) < 4)
                    pVal = .08;

                power = (sensors.getDistanceL() - 48) * -pVal;

                if (Math.abs(power) < lowThresh && power > 0){
                    power = lowThresh;

                } else if ((power < 0 && Math.abs(power) < lowThresh)){
                    power = -lowThresh;
                }

                drivetrainM.strafe(power, power, 0, 90);
            }
        }
        if (vu == 2) {


            drivetrainM.strafe(-.6, -.6, 0, 90);

            Thread.sleep(500);

                    telemetry.update();
//                drivetrainM.strafe(-.5, -.5, 0, 90);
                while ( Math.abs(sensors.getDistanceL() - 65) > .5 && (runtime.seconds() < 5) && sensors.getDistanceL() != 0 && !isStopRequested())
                {
                    power = (sensors.getDistanceL() - 65) * -.03;

                    if (Math.abs(power) < lowThresh && power > 0){
                        power = lowThresh;
                    } else if ((Math.abs(power) < lowThresh && power < 0)){
                        power = -lowThresh;
                    }

                    drivetrainM.strafe(power, power, 0, 90);
                }
//            } while (((sensors.getDistanceL()) > 68.5 + 1 || sensors.getDistanceL() < 68.5 - 1) && (runtime.seconds() < 100) && sensors.getDistanceL() != 0 && !isStopRequested());

            drivetrainM.stopMotors();
        }
        if (vu == 3) {

            drivetrainM.strafe(-.5, -.5, 0, 90);
            Thread.sleep(500);

            while ( Math.abs(sensors.getDistanceL() - 87) > .5 && (runtime.seconds() < 5) && sensors.getDistanceL() != 0 && !isStopRequested())
            {
                power = (sensors.getDistanceL() - 87) * -.03;

                if (Math.abs(power) < lowThresh && power > 0){
                    power = lowThresh;
                } else if ((Math.abs(power) < lowThresh && power < 0)){
                    power = -lowThresh;
                }

                drivetrainM.strafe(power, power, 0, 90);
            }
        }

//        while (sensors.getGyroYaw() < -80 && !isStopRequested()){
//            drivetrainM.startMotors(-.2, .2);
//        }
//
        Thread.sleep(500);


        // Move forward and deposits
        drivetrainM.startMotors(.33, .3);

        Thread.sleep(300);

        glyphScorer.intakeOut(1);

        Thread.sleep(500);

        drivetrainM.stopMotors();

        Thread.sleep(300);

        // Back up
        drivetrainM.startMotors(-.3, -.3);

        Thread.sleep(400);

        drivetrainM.stopMotors();

        glyphScorer.intakeStop();

        drivetrainM.pid(.3, -90, .1, .0055, .002, 0, 1, 4);

        if (sensors.getGyroYaw() < -95)
        {
            while (sensors.getGyroYaw() < -92){
                drivetrainM.startMotors(-.2, .2);
            }
        }

        drivetrainM.stopMotors();

        Thread.sleep(100);

        runtime.reset();

        lowThresh = .35;

        if (vu == 1) {

            while ( Math.abs(sensors.getDistanceR() - 48) > .5 && (runtime.seconds() < 5) && sensors.getDistanceR() != 0 && !isStopRequested())
            {
                double pVal = .08;

                if (Math.abs(sensors.getDistanceR() - 48) > 6)
                    pVal = .03;
                power = (sensors.getDistanceR() - 48) * -pVal;

                if (Math.abs(power) < lowThresh && power > 0){
                    power = lowThresh;
                } else if ((Math.abs(power) < lowThresh && power < 0)){
                    power = -lowThresh;
                }

                drivetrainM.strafeLeft(-power, -power, 0, -90);
            }
        }
        if (vu == 2) {

            double pVal = .08;

            telemetry.update();
//                drivetrainM.strafe(-.5, -.5, 0, 90);
            while ( Math.abs(sensors.getDistanceR() - 65) > .5 && (runtime.seconds() < 5) && sensors.getDistanceR() != 0 && !isStopRequested())
            {
                if (Math.abs(sensors.getDistanceR() - 65) > 6)
                    pVal = .03;
                power = (sensors.getDistanceR() - 65) * -pVal;

                if (Math.abs(power) < lowThresh && power > 0){
                    power = lowThresh;
                } else if ((Math.abs(power) < lowThresh && power < 0)){
                    power = -lowThresh;
                }

                drivetrainM.strafeLeft(-power, -power, 0, -90);
            }
//            } while (((sensors.getDistanceL()) > 68.5 + 1 || sensors.getDistanceL() < 68.5 - 1) && (runtime.seconds() < 100) && sensors.getDistanceL() != 0 && !isStopRequested());

            drivetrainM.stopMotors();
        }
        if (vu == 3) {

            double pVal = .08;

            while ( Math.abs(sensors.getDistanceR() - 87) > .5 && (runtime.seconds() < 5) && sensors.getDistanceR() != 0 && !isStopRequested())
            {
                if (Math.abs(sensors.getDistanceR() - 87) > 6)
                    pVal = .03;
                power = (sensors.getDistanceR() - 87) * -pVal;

                if (Math.abs(power) < lowThresh && power > 0){
                    power = lowThresh;
                } else if ((Math.abs(power) < lowThresh && power < 0)){
                    power = -lowThresh;
                }

                drivetrainM.strafeLeft(-power, -power, 0, -90);
            }
        }

//        Move forward and deposit
        drivetrainM.startMotors(-.3, -.33);

        Thread.sleep(500);
//
        glyphScorer.outputOut();
//
        Thread.sleep(1200);
//
//        // Back up to park
        drivetrainM.startMotors(.2, .2);

        Thread.sleep(500);

        drivetrainM.stopMotors();

        Thread.sleep(50);

        drivetrainM.startMotors(-.4, -.4);

        Thread.sleep(300);

        drivetrainM.stopMotors();

        Thread.sleep(200);

        drivetrainM.startMotors(.5, .5);

        Thread.sleep(200);

        glyphScorer.stopOutput();

        drivetrainM.stopMotors();

//        Thread.sleep(500);
//
//        drivetrainM.startMotors(.5, .5);
//
//        Thread.sleep(500);
//
//        drivetrainM.stopMotors();
//
//        Thread.sleep(200);
//
//        drivetrainM.startMotors(-.25, -.25);
//
//        Thread.sleep(250);
//
//        glyphScorer.intakeStop();
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
        telemetry.addLine()
                .addData("ColorL", new Func<String>() {
                    @Override public String value() {
                        return "ColorL " + sensors.getBlue();
                    }
                });


        telemetry.addLine()
                .addData("gyroPitch", new Func<String>() {
                    @Override public String value() {
                        return "gyro pitch: " + drivetrainM.sensor.getGyroPitch();
                    }
                });
        telemetry.addLine()
                .addData("gyroRoll", new Func<String>() {
                    @Override public String value() {
                        return "gyro roll: " + drivetrainM.sensor.getGyroRoll();
                    }
                });
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
                .addData("distanceF", new Func<String>() {
                    @Override public String value() {
                        return "distF:" + sensors.getDistanceF();
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

        while (vuMark == RelicRecoveryVuMark.UNKNOWN && !isStarted()) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
        }
        telemetry.addData("VuMark ", vuMark);

            if (vuMark == RelicRecoveryVuMark.RIGHT) {
                vu = 3;
            } else if (vuMark == RelicRecoveryVuMark.LEFT) {
                vu = 1;
            } else {
                vu = 2;
        }

    }

}



