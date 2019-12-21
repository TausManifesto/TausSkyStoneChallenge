//code for the autonomous portion of the Skystone challenge

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 * created by ashwin jandhyala
 * 10/6/2019
 */
//@Disabled
@Autonomous(name = "Foundation_Red_Wall", group = "Tau")

public class Auto_Mechanum_Foundation_Red_Wall extends Auto_Methods {

    @Override
    public void runOpMode() throws InterruptedException {

        //initializing robot
        initRobot();

        //after start is pressed

        //red -- foundation for adjacent to wall
        liftUp(.5, 1000);
        strafeLeft(.5,8);
        backward(.5, 35);
        liftDrop();
        forward(.5, 27);
        right(90);
        liftUp(.5,1000);
        strafeLeft(.5,10);
        forward(.5,15);
        liftDrop();
        forward(.5,25);




    }
}