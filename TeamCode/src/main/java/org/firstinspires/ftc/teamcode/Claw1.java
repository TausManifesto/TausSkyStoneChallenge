package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;



@Autonomous(name="Claw Test", group="Taus")

public class Claw1  extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private CRServo clawServo = null;

    static final double COUNTS_PER_MOTOR = 0.0;




    @Override
    public void runOpMode() {

        clawServo = hardwareMap.crservo.get("intake_servo");
        clawServo.setPower(0);

        waitForStart();



        while (opModeIsActive())  {

            telemetry.addData("Its starting", "grab");
            telemetry.update();
            grabBlock("grab");
            sleep(10000);
            telemetry.addData("Its closing", "release");
            telemetry.update();
            telemetry.addData("motor encoder values", "testMotor");
            break;

        }

    }


    public void grabBlock (String type) {


        if (type.equalsIgnoreCase("grab")) {

            clawServo.setPower(0.3);
            sleep(1000);


            clawServo.setPower(0);


        } else if (type.equalsIgnoreCase("release")) {

            clawServo.setPower(-0.3);
            sleep(500);
            clawServo.setPower(0);


        }


    }

    // degrees = encoder_value * (360/encodervalueofticks)%360



}
