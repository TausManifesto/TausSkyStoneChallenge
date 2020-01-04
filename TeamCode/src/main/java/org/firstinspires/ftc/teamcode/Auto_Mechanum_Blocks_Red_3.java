//code for the autonomous portion of the Skystone challenge

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 * created by ashwin jandhyala
 * 10/6/2019
 */

@Autonomous(name = "Blocks_Red_3", group = "Tau")

public class Auto_Mechanum_Blocks_Red_3 extends Vision_Test {


    @Override
    public void runOpMode() throws InterruptedException {

        String position = SkystonePos("red");

        telemetry.addData("pos", position);

        telemetry.update();

        switch (position) {
            case "Right":
                strafeRight(.4, 7);
                break;
            case "Center":
                strafeLeft(.4, 1);
                break;
            case "Left":
                strafeLeft(.4, 9);
                break;
        }
        outtake();
        sleep(500);
        intake();
        adjust(0);
        forward(.4, 30);
        adjust(0);
        backward(.4, 30);
        stopIntake();
        adjust(0);
        switch (position) {
            case "Right":
                strafeRight(.4, 51);
                break;
            case "Center":
                strafeRight(.4, 59);
                break;
            case "Left":
                strafeRight(.4, 67);
                break;
        }
        outtake();
        sleep(2000);
        stopIntake();
        strafeLeft(.7, 20);
    }
}