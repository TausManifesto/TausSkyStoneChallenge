//code for the autonomous portion of the Skystone challenge

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


/**
 * created by ashwin jandhyala
 * 10/6/2019
 */

@Disabled
@Autonomous(name = "Blocks_Blue_2", group = "Tau")

public class Auto_Mechanum_Blocks_Blue_2 extends Vision_Test {


    @Override
    public void runOpMode() throws InterruptedException {

        String position = SkystonePos("blue");

        telemetry.addData("pos", position);

        telemetry.update();

        switch (position) {
            case "Left":
                strafeRight(1, 15);
                break;
            case "Center":
                strafeRight(1, 23);
                break;
            case "Right":
                strafeRight(1, 31);
                break;
        }
        outtake();
        sleep(100);
        intake();
        forward(1, 30);
        backward(1, 30);
        stopIntake();
        switch (position) {
            case "Right":
                strafeLeft(1, 51);
                break;
            case "Center":
                strafeLeft(1, 59);
                break;
            case "Left":
                strafeLeft(1, 67);
                break;
        }
        outtake();
        sleep(1000);
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
        intake();
        forward(1, 30);
        intake2();
        backward(1, 30);
        stopIntake();
        switch (position) {
            case "Right":
                strafeLeft(1, 75);
                break;
            case "Center":
                strafeLeft(1, 83);
                break;
            case "Left":
                strafeLeft(1, 87);
                break;
        }
        outtake();
        sleep(500);
        stopIntake();
        strafeRight(1, 20);
    }
}