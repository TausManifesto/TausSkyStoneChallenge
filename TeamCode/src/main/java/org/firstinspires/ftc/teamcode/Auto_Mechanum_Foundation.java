//code for the autonomous portion of the Skystone challenge

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 * created by ashwin jandhyala
 * 10/6/2019
 */
//@Disabled
@Autonomous(name = "Foundation_Test", group = "Tau")

public class Auto_Mechanum_Foundation extends Auto_Methods {

    @Override
    public void runOpMode() throws InterruptedException {

        //initializing robot
        initRobot();

        //after start is pressed-

        //red -- foundation
        liftUp(.5, 1000);
        backward(.5, 35);
        liftDrop();
        forward(.5, 35);
        right(90);
        liftUp(.5, 1000);
        backward(.2, 35);
        liftDrop();
        backward(.2, 20);


    }
}