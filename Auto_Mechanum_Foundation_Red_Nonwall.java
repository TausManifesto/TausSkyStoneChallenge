//code for the autonomous portion of the Skystone challenge

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 * created by ashwin jandhyala
 * 10/6/2019
 */
//@Disabled
@Autonomous(name = "Foundation_Red_Nonwall", group = "Tau")

public class Auto_Mechanum_Foundation_Red_Nonwall extends Auto_Methods {

    @Override
    public void runOpMode() throws InterruptedException {

        //initializing robot
        initRobot();

        //after start is pressed

        //red -- foundation for adjacent to middle bridge
        liftUp(.5, 1000);
        strafeLeft(.5,3);
        backward(.5, 35);
        liftDrop();
        forward(.5, 17);
        strafeRight(.5,6);
        right(90);
        liftUp(.5, 1000);
        backward(.5, 13);
        forward(.5,15);
        liftDrop();
        forward(.5,25);
    }
}