//code for the autonomous portion of the Skystone challenge

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


/**
 * created by ashwin jandhyala
 * 10/6/2019
 */
@Disabled
@Autonomous(name = "2019_FTC_Auto_Blocks_Blue", group = "Tau")

public class Auto_Mechanum_Blocks_Blue extends Auto_Methods {


    @Override
    public void runOpMode() throws InterruptedException {

        //initializing robot
        initRobot();

        //after start is pressed

        String position = getSkystonePosBlue();

        //NO INTAKE

        //blue -- blocks -- no intake
        forward(.7, 30);
        gotoSkystone2("Blue", position);
        forward(.7, 50);
        backward(.7, 25);

    }
}