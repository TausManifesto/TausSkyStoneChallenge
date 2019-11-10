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
@TeleOp(name = "Drivercentric2", group = "Taus")
public class TeleOp_DriverCentric2 extends LinearOpMode {

    float rotate_angle = 0;
    double reset_angle = 0;
    BNO055IMU imu;
    private Hardware robot = null;
    private double liftPower = 0;
    private double extensionPower = 0;
    private double leftGP1X;
    private double rightGP1X;
    private double leftGP1Y;
    private double maxPOWER;
    private double frontleftPOWER;
    private double frontrightPOWER;
    private double backleftPOWER;
    private double backrightPOWER;


    @Override
    public void runOpMode() {
        robot = new Hardware();

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


        while (!opModeIsActive()) {
        }

        while (opModeIsActive()) {

            resetAngle();

            drive2();

            //Pressing gamepad2.x is for closing claw and gamepad2.A is for opening claw
            claw();

            //gamepad2 left trigger is for extending or retracting
            extend();

            //gamepad2 trigger is for lift going up and down.
            lift();

        }
    }


    public void drive() {
        double Protate = Math.abs(gamepad1.right_stick_x) <= 0.07 ? 0 : gamepad1.right_stick_x / 4;
        double stick_x = Math.abs(gamepad1.left_stick_x) <= 0.07 ? 0 : gamepad1.left_stick_x * Math.sqrt(Math.pow(1 - Math.abs(Protate), 2) / 2); //Accounts for Protate when limiting magnitude to be less than 1
        double stick_y = Math.abs(gamepad1.left_stick_y) <= 0.07 ? 0 : gamepad1.left_stick_y * Math.sqrt(Math.pow(1 - Math.abs(Protate), 2) / 2);
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


        //MOVEMENT for rotation
        theta = Math.atan2(stick_y, stick_x) - gyroAngle - (Math.PI / 2);
        Px = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * (Math.sin(theta + Math.PI / 4));
        Py = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * (Math.sin(theta - Math.PI / 4));


        robot.frontLeftMotor.setPower(Py - Protate);
        robot.backLeftMotor.setPower(Px - Protate);
        robot.backRightMotor.setPower(Py + Protate);
        robot.frontRightMotor.setPower(Px + Protate);
    }

    public void drive2() {
        leftGP1X = gamepad1.left_stick_x;
        leftGP1Y = -gamepad1.left_stick_y;
        rightGP1X = gamepad1.right_stick_x;

        if (Math.abs(leftGP1Y) < 0.05) {
            leftGP1Y = 0;
        }
        if (Math.abs(leftGP1X) < 0.05) {
            leftGP1X = 0;
        }

        if (Math.abs(rightGP1X) < 0.05) {
            rightGP1X = 0;
        }


        maxPOWER = Math.abs(frontleftPOWER);

        if (Math.abs(backleftPOWER) > maxPOWER) {
            maxPOWER = Math.abs(backleftPOWER);
        }
        if (Math.abs(backrightPOWER) > maxPOWER) {
            maxPOWER = Math.abs(backrightPOWER);
        }
        if (Math.abs(frontrightPOWER) > maxPOWER) {
            maxPOWER = Math.abs(frontrightPOWER);
        }

        if (maxPOWER > 1.0) {
            frontrightPOWER = frontrightPOWER / maxPOWER;
            frontleftPOWER = frontleftPOWER / maxPOWER;
            backrightPOWER = backrightPOWER / maxPOWER;
            backleftPOWER = backleftPOWER / maxPOWER;
        }


        robot.frontLeftMotor.setPower(frontleftPOWER);
        robot.frontRightMotor.setPower(frontrightPOWER);
        robot.backLeftMotor.setPower(backleftPOWER);
        robot.backRightMotor.setPower(backrightPOWER);
    }

    public void resetAngle() {
        if (gamepad1.a) {
            reset_angle = getHeading() + reset_angle;
        }
    }

    public double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
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

        if (gamepad2.x) {
            robot.clawServo.setPower(0.5);
        } else if (gamepad2.a) {
            robot.clawServo.setPower((-0.5));
            sleep(1000);
            robot.clawServo.setPower(-0.1);
        }

    }

    public void extend() {

        if (Math.abs(gamepad2.right_trigger) > 0.05) {
            telemetry.addData("extension", gamepad2.right_trigger);
            extensionPower = 0.4;
        } else if (gamepad2.right_bumper) {
            extensionPower = -0.4;
        } else {
            extensionPower = 0;
        }
        robot.leftExtensionServo.setPower(extensionPower);
        robot.rightExtensionServo.setPower(extensionPower);

    }

    public void lift() {

        if (Math.abs(gamepad2.left_trigger) > 0.07 && !robot.rightLimitSwitch.isPressed()) {
            telemetry.addData("lift", gamepad2.left_trigger);
            liftPower = 0.5;
        } else if (gamepad2.left_bumper) {
            liftPower = -0.1;
        }
        //else if (robot.rightLimitSwitch.isPressed()) {
        //    liftPower = 0;
        //}
        else {
            liftPower = .1;
        }

        robot.leftLiftServo.setPower(liftPower);
        robot.rightLiftServo.setPower(liftPower);

    }
}