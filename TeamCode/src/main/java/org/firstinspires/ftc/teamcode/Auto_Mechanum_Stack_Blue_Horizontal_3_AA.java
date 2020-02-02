package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "Stack_Horizontal_Blue_3", group = "Tau")


public class Auto_Mechanum_Stack_Blue_Horizontal_3_AA extends Vision_Test {

    @Override
    public void runOpMode() throws InterruptedException {

        String position = SkystonePos("blue");

        telemetry.addData("pos", position);

        telemetry.update();

        switch (position) {
            case "Left":
                strafeLeft(.5, 15);
                break;
            case "Center":
                strafeLeft(.5, 23);
                break;
            case "Right":
                strafeLeft(.5, 31);
                break;
        }
        outtake();
        sleep(100);
        intake();
        forward(.5, 30);
        intake2();
        backward(.7, 30);
        stopIntake();
        switch (position) {
            case "Right":
                strafeLeft(.5, 51);
                break;
            case "Center":
                strafeLeft(.5, 59);
                break;
            case "Left":
                strafeLeft(.5, 67);
                break;
        }
        forward(1, 10);
        strafeRight(1, 7.5);
        outtake();
        sleep(2000);
        stopIntake();
        backward(1, 22);
    }


}
