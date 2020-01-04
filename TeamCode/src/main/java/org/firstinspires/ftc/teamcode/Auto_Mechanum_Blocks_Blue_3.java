//code for the autonomous portion of the Skystone challenge

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 * created by ashwin jandhyala
 * 10/6/2019
 */

@Autonomous(name = "Blocks_Blue_3", group = "Tau")

public class Auto_Mechanum_Blocks_Blue_3 extends Vision_Test {


    @Override
    public void runOpMode() throws InterruptedException {

        String position = SkystonePos("blue");

        telemetry.addData("pos", position);

        telemetry.update();

        switch (position) {
            case "Left":
                strafeLeft(.4, 15);
                break;
            case "Center":
                strafeLeft(.4, 23);
                break;
            case "Right":
                strafeLeft(.4, 31);
                break;
        }
        outtake();
        sleep(100);
        intake();
        forward(.4, 30);
        stopIntake();
        backward(.4, 30);
        switch (position) {
            case "Right":
                strafeLeft(.4, 51);
                break;
            case "Center":
                strafeLeft(.4, 59);
                break;
            case "Left":
                strafeLeft(.4, 67);
                break;
        }
        outtake();
        sleep(2000);
        stopIntake();
        strafeRight(20, .7);
    }
}