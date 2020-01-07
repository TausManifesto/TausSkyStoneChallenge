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
        sleep(500);
        intake();
        forward(.5, 30);
        adjust(0);
        intake2();
        backward(.5, 30);
        stopIntake();
        adjust(0);
        switch (position) {
            case "Right":
                strafeRight(.5, 51);
                break;
            case "Center":
                strafeRight(.5, 59);
                break;
            case "Left":
                strafeRight(.5, 67);
                break;
        }
        outtake();
        sleep(2000);
        stopIntake();
        strafeLeft(.7, 20);
    }
}