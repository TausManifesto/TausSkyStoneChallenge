//code for the autonomous portion of the Skystone challenge

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


/**
 * created by ashwin jandhyala
 * 10/6/2019
 */
@Disabled
@Autonomous(name = "Blocks_Red_2", group = "Tau")

public class Auto_Mechanum_Blocks_Red_2 extends Vision_Test {


    @Override
    public void runOpMode() throws InterruptedException {

        String position = SkystonePos("red");

        telemetry.addData("pos", position);

        telemetry.update();

        switch (position) {
            case "Right":
                strafeRight(1, 7);
                break;
            case "Center":
                strafeLeft(1, 0);
                break;
            case "Left":
                strafeLeft(1, 8);
                break;
        }
        outtake();
        sleep(500);
        intake();
        forward(1, 30);
        backward(1, 30);
        stopIntake();
        switch (position) {
            case "Right":
                strafeRight(1, 31);
                outtake();
                strafeRight(1, 20);
                break;
            case "Center":
                strafeRight(1, 59);
                break;
            case "Left":
                strafeRight(1, 67);
                break;
        }
        outtake();
        sleep(500);

        switch (position) {
            case "Right":
                strafeLeft(1, 75);
                break;
            case "Center":
                strafeLeft(1, 83);
                break;
            case "Left":
                strafeLeft(1, 85);
                break;
        }
        intake();
        forward(.7, 33);
        intake2();
        backward(.7, 33);
        stopIntake();
        switch (position) {
            case "Right":
                strafeRight(1, 75);
                break;
            case "Center":
                strafeRight(1, 83);
                break;
            case "Left":
                strafeRight(1, 87);
                break;
        }
        outtake();
        sleep(500);
        strafeLeft(1, 20);

    }
}