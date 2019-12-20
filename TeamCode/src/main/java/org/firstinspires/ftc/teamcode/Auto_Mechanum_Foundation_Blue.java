//code for the autonomous portion of the Skystone challenge

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 * created by ashwin jandhyala
 * 10/6/2019
 */
//@Disabled
@Autonomous(name = "Foundation_Blue", group = "Tau")

public class Auto_Mechanum_Foundation_Blue extends Auto_Methods {

    @Override
    public void runOpMode() throws InterruptedException {

        //initializing robot
        initRobot();

        //after start is pressed

        //blue -- foundation
        liftUp(.5, 1000);
        strafeRight(.2, 15);
        backward(.5, 35);
        liftDrop();
        forward(.5, 35);
        liftUp(.5, 1000);
        strafeLeft(.2, 35);
        liftDrop();
        strafeLeft(.2, 20);


    }
}