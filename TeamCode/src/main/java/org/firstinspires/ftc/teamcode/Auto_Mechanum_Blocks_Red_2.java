//code for the autonomous portion of the Skystone challenge

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 * created by ashwin jandhyala
 * 10/6/2019
 */

@Autonomous(name = "Blocks_Red_2", group = "Tau")

public class Auto_Mechanum_Blocks_Red_2 extends Vision_Test {


    @Override
    public void runOpMode() throws InterruptedException {

        String position = SkystonePos("red");

        telemetry.addData("pos", position);

        telemetry.update();

        switch (position) {
            case "Right":
                strafeRight(.5, 7);
                break;
            case "Center":
                strafeLeft(.5, 1);
                break;
            case "Left":
                strafeLeft(.5, 9);
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
                strafeRight(8, 51);
                break;
            case "Center":
                strafeRight(.7, 59);
                break;
            case "Left":
                strafeRight(.7, 67);
                break;
        }
        outtake();
        sleep(2000);
        stopIntake();
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
                strafeRight(.7, 75);
                break;
            case "Center":
                strafeRight(.7, 83);
                break;
            case "Left":
                strafeRight(.7, 87);
                break;
        }
        outtake();
        sleep(2000);
        stopIntake();
        //adjust(0);
        strafeLeft(.7, 20);
    }
}