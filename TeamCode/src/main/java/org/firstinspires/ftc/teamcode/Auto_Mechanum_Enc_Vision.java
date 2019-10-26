//code for the autonomous portion of the Skystone challenge

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 * created by ashwin jandhyala
 * 10/6/2019
 */
//@Disabled
@Autonomous(name = "2019_FTC_Auto_Enc_Vision", group = "Tau")

public class Auto_Mechanum_Enc_Vision extends Auto_Methods {


    @Override
    public void runOpMode() throws InterruptedException {

        //initializing robot
        initRobot();

        //after start is pressed

        String position = getSkystonePos();

        //red -- foundation
        //forward(.4, 30);
        //sleep(1000);
        //backward(.4, 22);
        //sleep(1000);
        //strafeLeft(.4, 50);
        //sleep(1000);

        //red -- blocks 1 -- inside
        //gotoSkystone("Red");
        //backward(.7,56);
        //forward(.7, 30);

        //red -- blocks 2 -- inside
        //gotoSkystone("Red");
        //backward(.7,56);
        //forward(.7, 93);
        //right(90);
        //gotoSkystone("Red");
        //backward(.7, 88);


        //blue -- foundation
        //forward(.4, 30);
        //sleep(1000);
        //backward(.4, 22);
        //sleep(1000);
        //strafeRight(.4, 50);
        //sleep(1000);

        //blue -- blocks 1 -- inside
        //gotoSkystone("Blue");
        //backward(.7, 56);
        //forward(.7, 30);

        //blue -- blocks 2 -- inside
        gotoSkystone("Blue", position);
        backward(.2, 56);
        forward(.2, 80);
        left(90);
        backward(.2, 25);
        gotoSkystone2("Blue", position);
        backward(.2, 88);
        forward(.2, 30);


    }
}