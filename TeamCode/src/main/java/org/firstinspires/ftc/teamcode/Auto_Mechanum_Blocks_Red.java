//code for the autonomous portion of the Skystone challenge

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


/**
 * created by ashwin jandhyala
 * 10/6/2019
 */

@Autonomous(name = "2019_FTC_Auto_Blocks_Red", group = "Tau")

public class Auto_Mechanum_Blocks_Red extends Vision_Test {


    @Override
    public void runOpMode() throws InterruptedException {

        String position = SkystonePos("red");

        telemetry.addData("pos", position);

        telemetry.update();

        right(90);

        switch(position){
            case "Right":
                backward(.3, 12);
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
        forward(.3, 3);
        sleep(1000);
        stopIntake();
        strafeRight(.7, 19);
        switch(position){
            case "Right":
                forward(.7, 52);
                break;
            case "Center":
                forward(.7, 58);
                break;
            case "Left":
                forward(.7, 66);
                break;
        }
        outtake();
        sleep(1000);
        stopIntake();
        backward(.5, 5);



    }
}