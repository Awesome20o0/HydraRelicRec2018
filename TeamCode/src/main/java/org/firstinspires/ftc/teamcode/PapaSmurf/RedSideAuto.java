package org.firstinspires.ftc.teamcode.PapaSmurf;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Libraries.Drivetrain_Mecanum;
import org.firstinspires.ftc.teamcode.Libraries.GlyphScorer;
import org.firstinspires.ftc.teamcode.Libraries.JewelArm;
import org.firstinspires.ftc.teamcode.Libraries.SensorRR;

/**
 * Created by Varun on 9/11/2017.
 */

@Autonomous(name = "Red Side Auto", group = "LinearOpMode")
@Disabled
public class RedSideAuto extends LinearOpMode{
    private GlyphScorer glyphScorer;
    private Drivetrain_Mecanum drivetrainM;
    private String version;
    private SensorRR sensors;
    private JewelArm arm;
    public static final String TAG = "Vuforia VuMark Sample";


    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    @Override public void runOpMode() throws InterruptedException {

        drivetrainM = new Drivetrain_Mecanum(this);
        glyphScorer = new GlyphScorer(this);
        sensors = new SensorRR(this, true);
        arm = new JewelArm(this);

        composeTelemetry();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AYPVi+D/////AAAAGWcdhlXrGkdFvb06tBr5+AFjSDfw/YB3Am9Am/B21oh9Jy6CyrZzHhH1A7ssJo723Ha+8w0KNhmv38iW3hieiGS3ww/zbK7RgfMDhlAN5Ky/BZ2s2NUfKLIt32e9E6O23jOumaRs1Tw6BrIpfi0HnCjUwmkVi/Jd2FXUTvWOCPRiJ+Sm7J10sdb4612yzZnx/GpwnFsT9AtKamYqDzHs4CYDXlBJXetnon03SnnZjUxK/8NYbFRRIgKE+N/u3qCwSzus8GJkfwPbxMok9xIWwzrDnko2yiKqYb5wZlmZBYI722gR6IOmK8qlGJ+f+stBPQyseR7Q468By8u6WcucjveY3gVWh3uGbmzRE0BUTNkV";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        boolean right = false;
        boolean left = false;
        boolean center = false;

        ElapsedTime time = new ElapsedTime();


        waitForStart();

        relicTrackables.activate();

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        time.startTime();
        while ((vuMark == RelicRecoveryVuMark.UNKNOWN) && (time.milliseconds() < 2000)) {
        }

        telemetry.addData("VuMark", "%s visible", vuMark);

        if (vuMark == RelicRecoveryVuMark.CENTER)
            center = true;
        else if (vuMark == RelicRecoveryVuMark.LEFT)
            left = true;
        else
            right = true;

        OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
//        telemetry.addData("Pose", format(pose));

        if (pose != null) {
            VectorF trans = pose.getTranslation();
            Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            double tX = trans.get(0);
            double tY = trans.get(1);
            double tZ = trans.get(2);

            double rX = rot.firstAngle;
            double rY = rot.secondAngle;
            double rZ = rot.thirdAngle;
        } else {
            telemetry.addData("VuMark", "not visible");
        }


        telemetry.update();

        glyphScorer.liftUp();
        glyphScorer.liftStop();

        // 2. Extend arm
        arm.armOut();

        Thread.sleep(1500);

        int color = sensors.getColorValue();

        // 3. Knock ball off
        if (color < 0) {
            drivetrainM.strafepid(.5, 1000, .1, .0006, .00002, 0, 25, Math.PI, 2000);
            Thread.sleep(800);
            arm.armIn();
            Thread.sleep(500);
            drivetrainM.movepid(-.5, 1000, .1, .0001, .0005, 0, 10, 0, Math.PI / 2);
//            drivetrainM.movepid(.5, 1000, .1, .0001, .0005, 0, 10, 0, Math.PI / 2);

            // Works without the timeout, added a large value so we don't have to change anything
            drivetrainM.pid(1, 90, .18, 0.002, 0.0003, 0, 1, 5000);
            drivetrainM.movepid(.3, 800, .1, .0001, .0005, 0, 10, 0, Math.PI / 2);
//            drivetrainM.strafepid(.7, 1500, .1, .0002, .00006, 0, 25, 0, 5000);
//            drivetrainM.pid(1, -90, .18, 0.002, 0.0003, 0, 1);
//            drivetrainM.movepid(.5, 1000, .1, .0001, .0005, 0, 10, 0, Math.PI / 2);
//            Thread.sleep(500);
            Thread.sleep(1000);
            drivetrainM.strafepid(.5, 300, .1, .0006, .00002, 0, 25, Math.PI, 3000 );
            glyphScorer.outputOut();
            Thread.sleep(2500);
            drivetrainM.startMotors(.3, .3);
            Thread.sleep(300);
            drivetrainM.stopMotors();
        } else {
            //turn 15 degrees counterclockwise
            drivetrainM.pid(1, 15, .2, .008, 0.0003, 0, 1, 1500);
            Thread.sleep(500);
            arm.armIn();
//      4. Drive 24 inches off of balancing stone
            drivetrainM.movepid(.2, 1000, .1, .0004, .00003, 0, 25, 0, Math.PI / 2);
            Thread.sleep(1000);
//            5. Turn right in place
            drivetrainM.pid(1, 90, .18, 0.002, 0.00025, 0, 1, 1500);
            Thread.sleep(500);
            drivetrainM.movepid(.35, 600, .1, .0004, .00003, 0, 25, 0, Math.PI / 2);
            Thread.sleep(500);
            drivetrainM.pid(1, 90, .18, 0.002, 0.0003, 0, 1, 1500);
            Thread.sleep(500);
//            if (left) {
//                drivetrainM.strafepid(.7, 2500, .1, .00015, .00006, 0, 25, 0, 2500);
//                Thread.sleep(500);
//                drivetrainM.pid(1, -90, .15, 0.015, 0.0005, 0, 1, 1500);
//                Thread.sleep(500);
//                drivetrainM.movepid(.5, 1000, .1, .0001, .0005, 0, 10, 0, Math.PI / 2);
//            } else if (center) {
//                drivetrainM.strafepid(.7, 2000, .1, .00018, .00006, 0, 25, 0, 2500);
//                Thread.sleep(500);
//                drivetrainM.pid(1, -90, .15, 0.015, 0.0005, 0, 1, 1500);
//                Thread.sleep(500);
//                drivetrainM.movepid(.5, 1000, .1, .0001, .0005, 0, 10, 0, Math.PI / 2);
//
//            } else {
            drivetrainM.strafepid(.7, 1300, .1, .0002, .00006, 0, 25, Math.PI, 2500);
//                Thread.sleep(500);
//                drivetrainM.pid(1, -90, .15, 0.015, 0.0005, 0, 1, 1500);
//                Thread.sleep(500);
            drivetrainM.movepid(.3, 1000, .1, .0001, .0005, 0, 10, 0, Math.PI / 2);
//            }
            // 8. Manipulator deposits the glyphs into the cryptobox
            glyphScorer.outputOut();
            // 9. Wait for 1.5 seconds (while glyphs are being deposited)
            Thread.sleep(1500);
            // 10. Stop the manipulator
//            glyphScorer.stopOutput();
            drivetrainM.startMotors(.2, .2);
            Thread.sleep(600);
            drivetrainM.stopMotors();

            Thread.sleep(100);

            glyphScorer.stopOutput();

        }
    }

//    machineLearn = new MachineLearn;
//    machineLearn.start;

//    String format(OpenGLMatrix transformationMatrix){
//        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
//    }

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
    }

}

