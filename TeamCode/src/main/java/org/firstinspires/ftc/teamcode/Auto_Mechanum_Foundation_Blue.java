//code for the autonomous portion of the Skystone challenge

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 * created by ashwin jandhyala
 * 10/6/2019
 */
//@Disabled
@Autonomous(name = "2019_FTC_Auto_Foundation_Blue", group = "Tau")

public class Auto_Mechanum_Foundation_Blue extends Auto_Methods {

    @Override
    public void runOpMode() throws InterruptedException {

        //initializing robot
        initRobot();

        //after start is pressed

        //blue -- foundation
        backward(.4, 30);
        forward(.4, 27);
        sleep(10000);
        strafeLeft(.4, 55);

    }
}