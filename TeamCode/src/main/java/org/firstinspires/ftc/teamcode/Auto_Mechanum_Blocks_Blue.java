//code for the autonomous portion of the Skystone challenge

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 * created by ashwin jandhyala
 * 10/6/2019
 */
//@Disabled
@Autonomous(name = "2019_FTC_Auto_Blocks", group = "Tau")

public class Auto_Mechanum_Blocks_Blue extends Auto_Methods {


    @Override
    public void runOpMode() throws InterruptedException {

        //initializing robot
        initRobot();

        //after start is pressed

        String position = getSkystonePosRed();

        //NO INTAKE

        //blue -- blocks -- no intake
        gotoSkystone("Red", position);
        forward(.7, 50);
        backward(.7, 25);

    }
}