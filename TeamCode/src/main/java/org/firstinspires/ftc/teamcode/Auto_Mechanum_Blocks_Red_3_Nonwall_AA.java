
//code for the autonomous portion of the Skystone challenge

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "Stack_Vertical_Red_3_Nonwall", group = "Tau")

public class Auto_Mechanum_Blocks_Red_3_Nonwall_AA extends Vision_Test {


    @Override
    public void runOpMode() throws InterruptedException {

        String position = SkystonePos("red");

        telemetry.addData("pos", position);

        telemetry.update();

        switch (position) {
            case "Right":
                strafeRight(.4, 7);
                break;
            case "Center":
                strafeLeft(.4, 1);
                break;
            case "Left":
                strafeLeft(.4, 9);
                break;
        }


        outtake();
        sleep(500);
        intake();
        forward(.4, 30);
        backward(.7, 30);
        stopIntake();
        switch (position) {
            case "Right":
                strafeRight(.7, 51);
                break;
            case "Center":
                strafeRight(.7, 59);
                break;
            case "Left":
                strafeRight(.7, 67);
                break;
        }
        right(90);
        forward(1, 4.5);
        strafeRight(1, 3);
        outtake();
        sleep(2000);
        stopIntake();
        backward(0.8, 19.5);


    }
}