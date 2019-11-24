package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name = "DriveFinal", group = "Taus")
public class Drive_Tau extends LinearOpMode {

    public CRServo leftLiftServo = null;
    public CRServo rightLiftServo = null;
    public CRServo leftExtensionServo = null;
    public CRServo rightExtensionServo = null;
    double reset_angle = 0;
    boolean Xpressed = true;
    boolean XStillPressed = false;
    BNO055IMU imu;
    private DcMotor front_left_wheel = null;
    private DcMotor back_left_wheel = null;
    private DcMotor back_right_wheel = null;
    private DcMotor front_right_wheel = null;
    private DcMotor leftIntake = null;
    private DcMotor rightIntake = null;
    private double liftPower = 0;
    private double extensionPower = 0;
    private CRServo clawServo = null;
    private DigitalChannel leftLiftLimit = null;
    private DigitalChannel rightLiftLimit = null;
    private boolean fast = true;
    //private DigitalChannel leftExtensionLimit = null;
    //private DigitalChannel rightExtensionLimit = null;


    @Override
    public void runOpMode() {
        front_left_wheel = hardwareMap.dcMotor.get("left_front");
        back_left_wheel = hardwareMap.dcMotor.get("left_back");
        back_right_wheel = hardwareMap.dcMotor.get("right_back");
        front_right_wheel = hardwareMap.dcMotor.get("right_front");
        leftIntake = hardwareMap.dcMotor.get("left_intake");
        rightIntake = hardwareMap.dcMotor.get("right_intake");
        leftLiftServo = hardwareMap.crservo.get("left_lift");
        rightLiftServo = hardwareMap.crservo.get("right_lift");
        leftExtensionServo = hardwareMap.crservo.get("left_extension");
        rightExtensionServo = hardwareMap.crservo.get("right_extension");
        clawServo = hardwareMap.crservo.get("claw_servo");
        leftLiftLimit = hardwareMap.get(DigitalChannel.class, "left_lift_limit0");
        rightLiftLimit = hardwareMap.get(DigitalChannel.class, "right_lift_limit0");
        //leftExtensionLimit = hardwareMap.get(TouchSensor.class, "leftExtensionLimit");
        //rightExtensionLimit = hardwareMap.get(TouchSensor.class, "rightExtensionLimit");

        front_left_wheel.setDirection(DcMotor.Direction.REVERSE);
        back_left_wheel.setDirection(DcMotor.Direction.REVERSE);
        front_right_wheel.setDirection(DcMotor.Direction.FORWARD);
        back_right_wheel.setDirection(DcMotor.Direction.FORWARD);

        front_left_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftExtensionServo.setDirection(DcMotorSimple.Direction.REVERSE);
        rightExtensionServo.setDirection(DcMotorSimple.Direction.FORWARD);
        leftLiftServo.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLiftServo.setDirection(DcMotorSimple.Direction.FORWARD);

        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);


        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);

        while (!opModeIsActive()) {
        }


        leftExtensionServo.setPower(.3);
        rightExtensionServo.setPower(.3);
        sleep(1000);
        leftExtensionServo.setPower(0);
        rightExtensionServo.setPower(0);

        while (opModeIsActive()) {

            drive();

            resetAngle();

            //Pressing gamepad2.x is for closing claw and gamepad2.A is for opening claw
            claw();

            //gamepad2 left trigger is for extending or retracting
            extend();

            //gamepad2 trigger is for lift going up and down.
            lift();

            intake();

            telemetry.addData("Right is pressed ", rightLiftLimit.getState());
            telemetry.update();
            telemetry.addData("Left is pressed ", leftLiftLimit.getState());
            telemetry.update();
        }
    }


    public void drive() {

        double scaleFactor = 1;
        double Protate = gamepad1.right_stick_x / 4;
        double stick_x = gamepad1.left_stick_x; //* Math.sqrt(Math.pow(1 - Math.abs(Protate), 2) / 2); //Accounts for Protate when limiting magnitude to be less than 1
        double stick_y = gamepad1.left_stick_y; //* Math.sqrt(Math.pow(1 - Math.abs(Protate), 2) / 2);
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
            stick_x = .3;
        } else if (gamepad1.dpad_left) {
            stick_x = -.3;
        }
        if (gamepad1.dpad_up) {
            stick_y = .3;
        } else if (gamepad1.dpad_down) {
            stick_y = -.3;
        }


        //MOVEMENT for rotation
        theta = Math.atan2(stick_y, stick_x) - gyroAngle - (Math.PI / 2);
        Px = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * (Math.sin(theta + Math.PI / 4));
        Py = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * (Math.sin(theta - Math.PI / 4));

        if (Py - Protate > 1) {
            scaleFactor = Math.abs(Py - Protate);
        }
        if (Py + Protate > 1 && Py + Protate > scaleFactor) {
            scaleFactor = Math.abs(Py + Protate);
        }

        front_left_wheel.setPower((Py - Protate) / scaleFactor);
        back_left_wheel.setPower((Px - Protate) / scaleFactor);
        back_right_wheel.setPower((Py + Protate) / scaleFactor);
        front_right_wheel.setPower((Px + Protate) / scaleFactor);
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

        if (gamepad2.x && !XStillPressed) {
            if (!Xpressed) {
                clawServo.setPower(-.2);
                sleep(500);
                clawServo.setPower(0);
                Xpressed = true;
            } else {
                clawServo.setPower(.5);
                sleep(1000);
                clawServo.setPower(.3);
                Xpressed = false;
            }
            XStillPressed = true;
        }
        if (!gamepad2.x) {
            XStillPressed = false;
        }
    }

    public void extend() {

        if (Math.abs(gamepad2.right_trigger) > 0.05) {
            extensionPower = -0.4;
        } else if (gamepad2.right_bumper) {
            extensionPower = 0.4;
        } else {
            extensionPower = 0.05;
        }
        leftExtensionServo.setPower(extensionPower);
        rightExtensionServo.setPower(extensionPower);

    }

    public void lift() {
        if (gamepad2.left_bumper) {
            liftPower = 0.7;
        } else if (Math.abs(gamepad2.left_trigger) > .07 && !leftLiftLimit.getState()) {
            liftPower = -0.1;

        } else if (!leftLiftLimit.getState()) {
            liftPower = 0.1;
        } else {
            liftPower = 0;
        }

        leftLiftServo.setPower(liftPower);
        rightLiftServo.setPower(liftPower);

    }

    public void intake() {
        if (gamepad1.left_trigger > .05) {
            rightIntake.setPower(2);
            leftIntake.setPower(2);
        } else if (gamepad1.right_trigger > .05) {
            rightIntake.setPower(-1);
            leftIntake.setPower(-1);
        } else {
            rightIntake.setPower(0);
            leftIntake.setPower(0);
        }
    }
}

