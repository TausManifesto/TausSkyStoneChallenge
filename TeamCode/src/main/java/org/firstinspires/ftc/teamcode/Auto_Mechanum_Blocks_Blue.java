//code for the autonomous portion of the Skystone challenge

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 * created by ashwin jandhyala
 * 10/6/2019
 */
@Autonomous(name = "2019_FTC_Auto_Blocks_Blue", group = "Tau")

public class Auto_Mechanum_Blocks_Blue extends Vision_Test {


    @Override
    public void runOpMode() throws InterruptedException {

        String position = SkystonePos("blue");

        telemetry.addData("pos", position);

        telemetry.update();

        left(90);

        switch(position){
            case "Left":
                backward(.3, 15);
                break;
            case "Center":
                backward(.3, 21);
                break;
            case "Right":
                backward(.3, 29);
                break;
        }

        strafeRight(.3, 20);
        outtake();
        sleep(100);
        intake();
        forward(.3, 3);
        sleep(1000);
        stopIntake();
        strafeLeft(.7, 19);
        switch(position){
            case "Left":
                forward(.7, 52);
                break;
            case "Center":
                forward(.7, 58);
                break;
            case "Right":
                forward(.7, 66);
                break;
        }
        outtake();
        sleep(1000);
        stopIntake();
        backward(.5, 5);
    }
}