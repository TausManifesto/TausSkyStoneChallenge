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
        backward(.3, 32);
        liftDrop();
        forward(.3, 30);
        liftUp(.5, 1000);
        //sleep(5000);
        strafeRight(.2, 40);
        liftDrop();
        strafeRight(.2, 20);


    }
}