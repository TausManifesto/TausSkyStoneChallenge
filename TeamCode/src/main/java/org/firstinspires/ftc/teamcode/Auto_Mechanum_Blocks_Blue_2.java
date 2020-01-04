//code for the autonomous portion of the Skystone challenge

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


/**
 * created by ashwin jandhyala
 * 10/6/2019
 */

@Disabled
@Autonomous(name = "Blocks_Blue_2", group = "Tau")

public class Auto_Mechanum_Blocks_Blue_2 extends Vision_Test {


    @Override
    public void runOpMode() throws InterruptedException {

        String position = SkystonePos("blue");

        telemetry.addData("pos", position);

        telemetry.update();

        switch (position) {
            case "Left":
                strafeLeft(.3, 15);
                break;
            case "Center":
                strafeLeft(.3, 23);
                break;
            case "Right":
                strafeLeft(.3, 31);
                break;
        }
        outtake();
        sleep(100);
        intake();
        //adjust(0);
        forward(.7, 30);
        sleep(1000);
        stopIntake();
        //adjust(0);
        backward(.7, 30);
        //adjust(0);
        switch (position) {
            case "Right":
                strafeLeft(.7, 51);
                break;
            case "Center":
                strafeLeft(.7, 59);
                break;
            case "Left":
                strafeLeft(.7, 67);
                break;
        }
        outtake();
        sleep(2000);
        stopIntake();
        //adjust(0);
        switch (position) {
            case "Right":
                strafeRight(.7, 75);
                break;
            case "Center":
                strafeRight(.7, 83);
                break;
            case "Left":
                strafeRight(.7, 87);
                break;
        }
        //adjust(0);
        intake();
        forward(.7, 30);
        sleep(1000);
        stopIntake();
        //adjust(0);
        backward(.7, 30);
        //adjust(0);
        switch (position) {
            case "Right":
                strafeLeft(.7, 75);
                break;
            case "Center":
                strafeLeft(.7, 83);
                break;
            case "Left":
                strafeLeft(.7, 87);
                break;
        }
        outtake();
        sleep(2000);
        stopIntake();
        //adjust(0);
        strafeRight(.7, 20);
    }
}