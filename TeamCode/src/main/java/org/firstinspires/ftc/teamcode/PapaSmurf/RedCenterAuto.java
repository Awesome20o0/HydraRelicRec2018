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
 * Created by Avi on 10/2/2017.
 */

@Autonomous(name = "Red Center Auto", group = "LinearOpMode")
@Disabled
public class RedCenterAuto extends LinearOpMode {
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

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AYPVi+D/////AAAAGWcdhlXrGkdFvb06tBr5+AFjSDfw/YB3Am9Am/B21oh9Jy6CyrZzHhH1A7ssJo723Ha+8w0KNhmv38iW3hieiGS3ww/zbK7RgfMDhlAN5Ky/BZ2s2NUfKLIt32e9E6O23jOumaRs1Tw6BrIpfi0HnCjUwmkVi/Jd2FXUTvWOCPRiJ+Sm7J10sdb4612yzZnx/GpwnFsT9AtKamYqDzHs4CYDXlBJXetnon03SnnZjUxK/8NYbFRRIgKE+N/u3qCwSzus8GJkfwPbxMok9xIWwzrDnko2yiKqYb5wZlmZBYI722gR6IOmK8qlGJ+f+stBPQyseR7Q468By8u6WcucjveY3gVWh3uGbmzRE0BUTNkV";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
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

        while (opModeIsActive()) {

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
            telemetry.addData("Pose", format(pose));

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

            // 2. Extend arm
            arm.armOut();

            // 3. Knock ball off
            if (sensors.getColorValue() > 0){
                //turn 15 degrees clockwise
                drivetrainM.pid(1, 15, .1, 0, 0, 0, 0, 0);
                arm.armIn();
                drivetrainM.pid(1, -15, .1, 0, 0, 0, 0, 0);
            }else{
                //turn 15 degrees counterclockwise
                drivetrainM.pid(1, -15, .1, 0, 0, 0, 0, 0);
                arm.armIn();
                drivetrainM.pid(1, 15, .1, 0, 0, 0, 0, 0);
            }

            // 4. Move left off the platform
            drivetrainM.movepid(1, 3000, .1, 0, 0, 0, 100, 0, Math.PI);

            // 5. Turn 180 degress in place
            drivetrainM.pid(1, 180, .1, 0, 0, 0, 0, 0);

            // 6. Move forward 20 inches towards cryptobox
            drivetrainM.movepid(1, 2500, .1, 0, 0, 0, 100, 0, 0);

            // 7. Move horizontally depending on vuMark value
            if(left){
                drivetrainM.movepid(1, 4000, .1, 0, 0, 0, 100, 0, 0);

            } else if (center) {
                drivetrainM.movepid(1, 3000, .1, 0, 0, 0, 100, 0, 0);

            } else {
                drivetrainM.movepid(1, 2000, .1, 0, 0, 0, 100, 0, 0);
            }

            // 8. Manipulator deposits the glyphs into the cryptobox
            glyphScorer.outputOut();

            // 9. Wait for 1.5 seconds (while glyphs are being deposited)
            Thread.sleep(1500);

            // 10. Stop the manipulator
            glyphScorer.stopOutput();

        }
    }

    String format(OpenGLMatrix transformationMatrix){
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    // Telemetry stuff
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



