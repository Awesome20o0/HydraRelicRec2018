package org.firstinspires.ftc.teamcode.PapaSmurf;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Libraries.Drivetrain_Mecanum;
import org.firstinspires.ftc.teamcode.Libraries.GlyphScorer;
import org.firstinspires.ftc.teamcode.Libraries.JewelArm;
import org.firstinspires.ftc.teamcode.Libraries.SensorRR;

/**
 * Created by willi on 11/6/2017.
 */
@Autonomous(name = "EZ BALANCE", group = "LinearOpMode")
public class OPMultiGlyphRed extends LinearOpMode {
    private GlyphScorer glyphScorer;
    private Drivetrain_Mecanum drivetrainM;
    private String version;
    private SensorRR sensors;
    private JewelArm arm;
    public static final String TAG = "Vuforia VuMark Test";
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;
    int vu = 3;
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

        // Move omnipulator up to prevent it from hitting the balancing stone
        glyphScorer.liftUp();

        double kP_FB = .0556;
        double kP_LR = .0234;

        double diffPitch = drivetrainM.sensor.getGyroPitch();
        double diffRoll = drivetrainM.sensor.getGyroRoll();

        double PIDchangeFB = -kP_FB* diffRoll;
        double PIDchangeLR = -kP_LR* diffPitch;

        double max = Math.max(Math.abs(PIDchangeFB+PIDchangeLR), Math.abs(PIDchangeFB - PIDchangeLR));

        max = max > 1 ? max : 1;

        drivetrainM.motorFL.setPower((PIDchangeFB + PIDchangeLR ) / max);
        drivetrainM.motorBL.setPower((PIDchangeFB + PIDchangeLR ) / max);
        drivetrainM.motorFR.setPower((PIDchangeFB - PIDchangeLR ) / max);
        drivetrainM.motorBR.setPower((PIDchangeFB - PIDchangeLR ) / max);
    }

    private void composeTelemetry() {
        telemetry.addLine()
                .addData("Avg", new Func<String>() {
                    @Override
                    public String value() {
                        return "avg: " + drivetrainM.getEncoderAvg();
                    }
                });
        telemetry.addLine()
                .addData("gyroYaw", new Func<String>() {
                    @Override
                    public String value() {
                        return "gyro yaw: " + drivetrainM.sensor.getGyroYaw();
                    }
                });
        telemetry.addLine()
                .addData("Color", new Func<String>() {
                    @Override
                    public String value() {
                        return "Color: " + sensors.getColorValue();
                    }
                });
        telemetry.addLine()
                .addData("Time", new Func<String>() {
                    @Override
                    public String value() {
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
                    @Override
                    public String value() {
                        return "VU:" + vu;
                    }
                });
        telemetry.addLine()
                .addData("distanceL", new Func<String>() {
                    @Override
                    public String value() {
                        return "distL:" + sensors.getDistanceL();
                    }
                });
        telemetry.addLine()
                .addData("distanceR", new Func<String>() {
                    @Override
                    public String value() {
                        return "distR:" + sensors.getDistanceR();
                    }
                });
    }
}



