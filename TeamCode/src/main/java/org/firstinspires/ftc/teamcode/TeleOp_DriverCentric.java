package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Disabled
@TeleOp(name="Drivercentric", group="Taus")
public class TeleOp_DriverCentric extends LinearOpMode {

    float rotate_angle = 0;
    double reset_angle = 0;
    double leftExtensionPower;
    double rightExtensionPower;
    double rightLiftPower;
    double leftLiftPower;

    BNO055IMU imu;
    Hardware robot = new Hardware();


    @Override
    public void runOpMode() {

        //initializing imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);

        robot.initTeleOpNOIMU(hardwareMap);


        telemetry.addLine("wait for gyro calibration"); //telling user to wait for gyro to be calibrated
        telemetry.update();

        //waiting for gyro to calibrate
        while (!imu.isGyroCalibrated()) {
            //waiting
        }

        telemetry.addData("gyro status :: ", imu.getCalibrationStatus().toString()); //returning gyro status (ready to start)
        telemetry.update();

        while(!opModeIsActive()){}

        while(opModeIsActive()){
            resetAngle();
            drive();

            // This commands are for claw functionality
            if(gamepad2.x ) {
                telemetry.addData("grabbing block", gamepad2.x);
                telemetry.update();
                grabBlock("grab");
            }
            if(gamepad2.b) {
                telemetry.addData("releasing block", gamepad2.b);
                telemetry.update();
                grabBlock("release");
            }


            //lift
            if(Math.abs(gamepad2.left_stick_x) > 0.05){
                telemetry.addData("extension", gamepad2.left_stick_x);
                //this will make sure we are using 70% power only
                if(Math.abs(gamepad2.left_stick_x) > 0.7){
                    //setting power based on movement of joystick right- extending, left - retracting
                    leftExtensionPower = gamepad2.left_stick_x > 0 ? 0.7 : -0.7 ;
                    rightExtensionPower = gamepad2.left_stick_x > 0 ? -0.7 : 0.7;
                } else {
                    leftExtensionPower = gamepad2.left_stick_x ;
                    rightExtensionPower = gamepad2.left_stick_x;
                }
                robot.leftExtensionServo.setPower(leftExtensionPower);
                robot.rightExtensionServo.setPower(rightExtensionPower);

            }


            if (Math.abs(gamepad2.right_stick_y) > 0.05 && !(robot.rightLimitSwitch.isPressed() || robot.leftLimitSwitch.isPressed())) {
                telemetry.addData("Stick_Y", gamepad2.right_stick_y);
                //this will make sure we are using 70% power only
                if(Math.abs(gamepad2.right_stick_y) > 0.7){
                    //setting power based on movement of joystick up - moving lift up, down - moving lift down
                    leftLiftPower = gamepad2.left_stick_y > 0 ? 0.7 : -0.7 ; ;
                    rightLiftPower = gamepad2.left_stick_y > 0 ? -0.7 : 0.7 ;;
                }else {
                    leftLiftPower = gamepad2.left_stick_y;
                    rightLiftPower = gamepad2.left_stick_y;
                }
                robot.leftLiftServo.setPower(leftLiftPower);
                robot.rightLiftServo.setPower(rightLiftPower);
            } else if (!robot.rightLimitSwitch.isPressed() || !robot.leftLimitSwitch.isPressed()) {
                robot.leftLiftServo.setPower(.1);
                robot.rightLiftServo.setPower(.1);
            } else {
                robot.leftLiftServo.setPower(0);
                robot.rightLiftServo.setPower(0);
            }


        }
    }



    public void drive() {

        double Protate = Math.abs(gamepad1.right_stick_x) > .05 ? 0 : gamepad1.right_stick_x / 4;
        double stick_x = Math.abs(gamepad1.left_stick_x) > .05 ? 0 : gamepad1.left_stick_x * Math.sqrt(Math.pow(1 - Math.abs(Protate), 2) / 2); //Accounts for Protate when limiting magnitude to be less than 1
        double stick_y = Math.abs(gamepad1.left_stick_y) > .05 ? 0 : gamepad1.left_stick_y * Math.sqrt(Math.pow(1 - Math.abs(Protate), 2) / 2);
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

        if(gamepad1.right_bumper){
            //Disables gyro, sets to -Math.PI/2 so front is defined correctly.
            gyroAngle = -Math.PI/2;
        }


        //MOVEMENT for rotation
        theta = Math.atan2(stick_y, stick_x) - gyroAngle - (Math.PI / 2);

        //if (Math.abs(stick_x)<.05){
        //    stick_x = 0;
        //}
        //if (Math.abs(stick_y)<.05){
        //    stick_y = 0;
        //}

        Px = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * (Math.sin(theta + Math.PI / 4));
        Py = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * (Math.sin(theta - Math.PI / 4));

        robot.frontLeftMotor.setPower(Py - Protate);
        robot.backLeftMotor.setPower(Px - Protate);
        robot.backRightMotor.setPower(Py + Protate);
        robot.frontRightMotor.setPower(Px + Protate);
    }

    public void resetAngle(){
        if(gamepad1.a){
            reset_angle = getHeading() + reset_angle;
        }
    }

    public double getHeading(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;
        if(heading < -180) {
            heading = heading + 360;
        }
        else if(heading > 180){
            heading = heading - 360;
        }
        heading = heading - reset_angle;
        return heading;
    }

    public void grabBlock (String type) {

        if (type.equalsIgnoreCase("grab")) {

            robot.clawServo.setPower(0.3);
            sleep(500);
            robot.clawServo.setPower(0.1);


        } else if (type.equalsIgnoreCase("release")) {

            robot.clawServo.setPower(-0.3);
            sleep(500);
            robot.clawServo.setPower(0);


        }


    }

}
