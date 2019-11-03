//code for the autonomous portion of the Skystone challenge

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 * created by ashwin jandhyala
 * 10/6/2019
 */
//@Disabled
@Autonomous(name = "2019_FTC_Auto_Blocks", group = "Tau")

public class Auto_Mechanum_Blocks extends Auto_Methods {


    @Override
    public void runOpMode() throws InterruptedException {

        //initializing robot
        initRobot();

        //after start is pressed

        String position = getSkystonePosRed();

        //NO INTAKE
        //red -- blocks -- no intake
        //gotoSkystone2("Red", position);
        //forward(.7,50);
        //backward(.7, 25);

        while (opModeIsActive()) {
            telemetry.addData("angle", getSkystonePosRed());
            telemetry.update();
        }
        //blue -- blocks -- no intake
        //gotoSkystone("Red", position);
        //forward(.7,50);
        //backward(.7, 25);


        //********************************************************//
        //RED


        //red -- blocks 1 -- inside
        //gotoSkystone("Red", position);
        //backward(.7,56);
        //forward(.7, 30);

        //red -- blocks 2 -- inside
        //gotoSkystone("Red", position);
        //backward(.7,56);
        //forward(.7, 80);
        //right(90);
        //gotoSkystone2("Red", position);
        //backward(.7, 88);
        //forward(.7, 30);

        //red -- blocks 1 -- outside
        //gotoSkystone("Red", position);
        //strafeLeft(.7, 24);
        //backward(.7,56);
        //strafeRight(.7, 24);
        //forward(.7, 30);

        //red -- blocks 2 -- outside
        //gotoSkystone("Red", position);
        //strafeLeft(.7, 24);
        //backward(.7,56);
        //strafeRight(.7, 24);
        //forward(.7, 80);
        //strafeLeft(.7, 24);
        //right(90);
        //gotoSkystone2("Red", position);
        //backward(.7, 88);
        //strafeRight(.7, 24)
        //forward(.7, 30);


        //**************************************//
        //BLUE


        //blue -- blocks 1 -- inside
        //gotoSkystone("Blue");
        //backward(.7, 56);
        //forward(.7, 30);

        //blue -- blocks 2 -- inside
        //gotoSkystone("Blue", position);
        //backward(.7, 56);
        //forward(.7, 80);
        //left(90);
        //gotoSkystone2("Blue", position);
        //backward(.7, 88);
        //forward(.7, 30);

        //blue -- blocks 1 -- outside
        //gotoSkystone("Blue", position);
        //strafeRight(.7, 24);
        //backward(.7,56);
        //strafeLeft(.7, 24);
        //forward(.7, 30);

        //blue -- blocks 2 -- outside
        //gotoSkystone("Blue", position);
        //strafeRight(.7, 24);
        //backward(.7,56);
        //strafeLeft(.7, 24);
        //forward(.7, 80);
        //strafeRight(.7, 24);
        //left(90);
        //gotoSkystone2("Blue", position);
        //backward(.7, 88);
        //strafeLeft(.7, 24);
        //forward(.7, 30);

    }
}