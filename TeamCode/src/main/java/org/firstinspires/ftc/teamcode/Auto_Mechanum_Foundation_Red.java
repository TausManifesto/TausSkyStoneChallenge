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

        //after start is pressed-

        //red -- foundation
        liftUp(.5, 1000);
        strafeLeft(.2, 12);
        backward(.3, 35);
        liftDrop();
        forward(.5, 35);
        right(10);
        liftUp(.5, 1000);
        sleep(1000);
        strafeRight(.2, 35);
        liftDrop();
        forward(.2, 5);
        strafeRight(.2, 20);


    }
}