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
 * Created by Avi on 11/1/2017.
 */

@Autonomous(name = "Devan Auto", group = "LinearOpMode")
@Disabled
public class DevanAuto extends LinearOpMode{
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
            //strafe right
            drivetrainM.strafepid(.5, 2000, .1, .0006, .00002, 0, 25, 0, 5000);
            Thread.sleep(1000);
            arm.armIn();
            Thread.sleep(1000);
            drivetrainM.strafepid(1, 1500, .1, .0002, .00006, 0, 25, 0, 5000);
            Thread.sleep(500);

//            Thread.sleep(1000);
//            drivetrainM.strafepid(.7, 1200, .1, .0002, .00006, 0, 25, 0, 5000);

        } else {
            //strafe left
            drivetrainM.strafepid(1, 2000, .1, .0002, .00002, 0, 25, Math.PI, 5000);
            Thread.sleep(500);
            arm.armIn();
//            Thread.sleep(1000);
//            drivetrainM.strafepid(.7, 1200, .1, .0002, .00006, 0, 25, Math.PI, 5000);

        }

        drivetrainM.movepid(-.5, 1000, .1, .0001, .0005, 0, 10, 0, Math.PI / 2);

        drivetrainM.movepid(.5, 1000, .1, .0001, .0005, 0, 10, 0, Math.PI / 2);

        drivetrainM.pid(1, -90, .18, 0.002, 0.0003, 0, 1, 10000);

        drivetrainM.movepid(.5, 800, .1, .0001, .0005, 0, 10, 0, Math.PI / 2);

        drivetrainM.strafepid(.7, 1500, .1, .0002, .00006, 0, 25, 0, 5000);

        drivetrainM.pid(1, -90, .18, 0.002, 0.0003, 0, 1, 10000);

        drivetrainM.movepid(.5, 1000, .1, .0001, .0005, 0, 10, 0, Math.PI / 2);

        Thread.sleep(500);

        // 8. Manipulator deposits the glyphs in to the cryptobox
        glyphScorer.outputOut();

        Thread.sleep(500);
        // 10. Stop the manipulator
        drivetrainM.movepid(.5, 100, .1, .001, .0005, 0, 10, 0, Math.PI * (3/2));

    }


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




