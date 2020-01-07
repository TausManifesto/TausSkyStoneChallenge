//code for the autonomous portion of the Skystone challenge

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 * created by ashwin jandhyala
 * 10/6/2019
 */

@Autonomous(name = "Blocks_Red", group = "Tau")

public class Auto_Mechanum_Blocks_Red extends Vision_Test {


    @Override
    public void runOpMode() throws InterruptedException {

        String position = SkystonePos("red");

        telemetry.addData("pos", position);

        telemetry.update();

        right(90);

        switch(position){
            case "Right":
                backward(.3, 10);
                break;
            case "Center":
                backward(.3, 18);
                break;
            case "Left":
                backward(.3, 26);
                break;
        }

        strafeLeft(.3, 20);
        outtake();
        sleep(100);
        intake();
        forward(.3, 5);
        sleep(1000);
        stopIntake();
        strafeRight(.7, 19.5);
        switch(position){
            case "Right":
                forward(.8, 51);
                break;
            case "Center":
                forward(.8, 59);
                break;
            case "Left":
                forward(.8, 67);
                break;
        }
        outtake();
        sleep(2000);
        stopIntake();
        backward(.7, 15);
        strafeLeft(1, 2);
    }
}