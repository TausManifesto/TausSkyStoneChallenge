package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import com.qualcomm.robotcore.hardware.TouchSensor;


@TeleOp(name = "Drivercentric", group = "Taus")
public class TeleOp_DriverCentric extends LinearOpMode {

    private Hardware robot = null;

    float rotate_angle = 0;
    double reset_angle = 0;


    private double liftPower = 0;
    private double extensionPower = 0;


    @Override
    public void runOpMode() {
        robot = new Hardware();
        robot.init(hardwareMap, telemetry);
<<<<<<< HEAD
        
=======

>>>>>>> first-game-latest
        robot.frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        robot.backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        robot.backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        robot.frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);


        while (!opModeIsActive()) {
        }

        while (opModeIsActive()) {
            drive();

            resetAngle();

            //Pressing gamepad2.x is for closing claw and gamepad2.A is for opening claw
            claw();

            //gamepad2 left trigger is for extending or retracting
            extend();

            //gamepad2 trigger is for lift going up and down.
            lift();

            telemetry.update();

        }
    }



    public void drive() {
        double Protate = Math.abs(gamepad1.right_stick_x) <= 0.07 ? 0 :  gamepad1.right_stick_x/ 4;
        double stick_x = Math.abs(gamepad1.left_stick_x) <= 0.07 ? 0 : gamepad1.left_stick_x * Math.sqrt(Math.pow(1 - Math.abs(Protate), 2) / 2); //Accounts for Protate when limiting magnitude to be less than 1
        double stick_y = Math.abs( gamepad1.left_stick_y) <= 0.07 ? 0 : gamepad1.left_stick_y * Math.sqrt(Math.pow(1 - Math.abs(Protate), 2) / 2);
        double theta = 0;
        double Px = 0;
        double Py = 0;

        double gyroAngle = getHeading() * Math.PI / 180; //Converts gyroAngle into radians
        if (gyroAngle <= 0) {
            gyroAngle = gyroAngle + (Math.PI / 2);
        } else if (0 < gyroAngle && gyroAngle < Math.PI / 2) {
            gyroAngle = gyroAngle + (Math.PI / 2);
        } else if (Math.PI / 2 <= gyroAngle) {
            gyroAngle = gyroAngle - (3 * Math.PI / 2);
        }
        gyroAngle = -1 * gyroAngle;

        if (gamepad1.right_bumper) {
            //Disables gyro, sets to -Math.PI/2 so front is defined correctly.
            gyroAngle = -Math.PI / 2;
        }

        //Linear directions in case you want to do straight lines.
        if (gamepad1.dpad_right) {
            stick_x = 0.5;
        } else if (gamepad1.dpad_left) {
            stick_x = -0.5;
        }
        if (gamepad1.dpad_up) {
            stick_y = -0.5;
        } else if (gamepad1.dpad_down) {
            stick_y = 0.5;
        }


        //MOVEMENT for rotation
        theta = Math.atan2(stick_y, stick_x) - gyroAngle - (Math.PI / 2);
        Px = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * (Math.sin(theta + Math.PI / 4));
        Py = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * (Math.sin(theta - Math.PI / 4));


        robot.frontLeftMotor.setPower(Py - Protate);
        robot.backLeftMotor.setPower(Px - Protate);
        robot.backRightMotor.setPower(Py + Protate);
        robot.frontRightMotor.setPower(Px + Protate);
    }


    public void resetAngle() {
        if (gamepad1.a) {
            reset_angle = getHeading() + reset_angle;
        }
    }

    public double getHeading() {
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;
        if (heading < -180) {
            heading = heading + 360;
        } else if (heading > 180) {
            heading = heading - 360;
        }
        heading = heading - reset_angle;
        return heading;
    }


    public void claw() {

        if(gamepad2.x ){
            robot.clawServo.setPower(0.2);
            telemetry.addData("grabbing block", gamepad2.x);
        } else if(gamepad2.a){
            telemetry.addData("releasing block", gamepad2.x);
            robot.clawServo.setPower((-0.2));
        }else {
            robot.clawServo.setPower(0.0);
        }

    }

    public void extend(){

        if (Math.abs(gamepad2.right_trigger) > 0.05) {
            telemetry.addData("extension", gamepad2.right_trigger);
            extensionPower = gamepad2.right_trigger > 0 ? 0.4 : -0.4;
        } else{
            extensionPower = 0.0;
        }
        robot.leftExtensionServo.setPower(extensionPower);
        robot.rightExtensionServo.setPower(extensionPower);

    }

    public void lift(){
        if (Math.abs(gamepad2.left_trigger) > 0.07 && !robot.leftLiftLimit.isPressed() && !robot.rightLiftLimit.isPressed()) {
            telemetry.addData("lift", gamepad2.left_trigger);
            liftPower = gamepad2.left_trigger > 0 ? 0.5 : -0.5;
<<<<<<< HEAD
                    ;
=======
            ;
>>>>>>> first-game-latest
        } else {
            liftPower = 0.0;
        }

        robot.leftLiftServo.setPower(liftPower);
        robot.rightLiftServo.setPower(liftPower);

    }
}