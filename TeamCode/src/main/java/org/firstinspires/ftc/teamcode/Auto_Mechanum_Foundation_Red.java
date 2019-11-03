//code for the autonomous portion of the Skystone challenge

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 * created by ashwin jandhyala
 * 10/6/2019
 */
//@Disabled
@Autonomous(name = "2019_FTC_Auto_Foundation_Red", group = "Tau")

public class Auto_Mechanum_Foundation_Red extends Auto_Methods {
    @Override
    public void runOpMode() throws InterruptedException {

        //initializing robot
        initRobot();

        //after start is pressed

        //red -- foundation
        backward(.4, 30);
        forward(.4, 27);
        sleep(10000);
        strafeRight(.4, 55);

    }
}