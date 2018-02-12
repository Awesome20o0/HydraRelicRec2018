package org.firstinspires.ftc.teamcode.AutonomousTesting;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.OldFiles.AutoMode;

/**
 * Created by Arib on 3/24/2016.
 */
@Disabled
public class CoordinateAuto extends AutoMode {


    @Override
    public void runOpMode() throws InterruptedException {
        waitOneFullHardwareCycle();
        cordFirst(2, 6, 1);
        waitOneFullHardwareCycle();

        waitForStart();

        waitOneFullHardwareCycle();

        moveToCoordinatePos(6, 3);

        waitOneFullHardwareCycle();

//        moveToCoordinatePos(3, 5);

//        waitOneFullHardwareCycle();


    }

}
