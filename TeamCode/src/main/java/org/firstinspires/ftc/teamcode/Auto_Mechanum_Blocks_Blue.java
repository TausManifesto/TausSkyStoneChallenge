//code for the autonomous portion of the Skystone challenge

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 * created by ashwin jandhyala
 * 10/6/2019
 */

@Autonomous(name = "Blocks_Blue", group = "Tau")

public class Auto_Mechanum_Blocks_Blue extends Vision_Test {


    @Override
    public void runOpMode() throws InterruptedException {

        String position = SkystonePos("blue");

        telemetry.addData("pos", position);

        telemetry.update();

        left(90);

        switch(position){
            case "Left":
                backward(.3, 15);
                break;
            case "Center":
                backward(.3, 23);
                break;
            case "Right":
                backward(.3, 31);
                break;
        }

        strafeRight(.3, 20);
        outtake();
        sleep(100);
        intake();
        forward(.3, 6);
        sleep(1000);
        stopIntake();
        strafeLeft(.7, 15);
        right(4);
        switch(position){
            case "Left":
                forward(.8, 53);
                break;
            case "Center":
                forward(.8, 59);
                break;
            case "Right":
                forward(.8, 67);
                break;
        }
        outtake();
        sleep(2000);
        stopIntake();
        backward(.7, 15);
        strafeRight(1, 2);
    }
}