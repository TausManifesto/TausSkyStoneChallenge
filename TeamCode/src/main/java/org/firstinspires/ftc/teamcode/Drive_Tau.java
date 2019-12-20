package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name = "DriveFinal", group = "Taus")
public class Drive_Tau extends LinearOpMode {

    public CRServo leftLiftServo = null;
    public CRServo rightLiftServo = null;
    public CRServo leftExtensionServo = null;
    double reset_angle = 0;
    boolean Xpressed = true;
    boolean XStillPressed = false;
    boolean X1pressed = false;
    boolean X1StillPressed = false;

    double turnFactor = 2;
    double driveFactor = 1;

    ElapsedTime period = new ElapsedTime();
    BNO055IMU imu;
    private DcMotor front_left_wheel = null;
    private DcMotor back_left_wheel = null;
    private DcMotor back_right_wheel = null;
    private DcMotor front_right_wheel = null;
    private DcMotor leftIntake = null;
    private DcMotor rightIntake = null;
    private CRServo intake2 = null;
    private CRServo intake3 = null;
    private double liftPower = 0;
    private double extensionPower = 0;
    private CRServo clawServo = null;
    //private DigitalChannel leftLiftLimit = null;
    private DigitalChannel rightLiftLimit = null;
    private DistanceSensor blockSensor = null;
    private DistanceSensor liftSensor = null;
    //private DigitalChannel leftExtensionLimit = null;
    private DigitalChannel rightExtensionLimit = null;


    @Override
    public void runOpMode() {
        front_left_wheel = hardwareMap.dcMotor.get("left_front");
        back_left_wheel = hardwareMap.dcMotor.get("left_back");
        back_right_wheel = hardwareMap.dcMotor.get("right_back");
        front_right_wheel = hardwareMap.dcMotor.get("right_front");
        leftIntake = hardwareMap.dcMotor.get("left_intake");
        rightIntake = hardwareMap.dcMotor.get("right_intake");
        intake2 = hardwareMap.crservo.get("intake_help_left");
        intake3 = hardwareMap.crservo.get("intake_help_right");
        leftLiftServo = hardwareMap.crservo.get("left_lift");
        rightLiftServo = hardwareMap.crservo.get("right_lift");
        leftExtensionServo = hardwareMap.crservo.get("left_extension");
        clawServo = hardwareMap.crservo.get("claw_servo");
        //leftLiftLimit = hardwareMap.get(DigitalChannel.class, "left_lift_limit0");
        rightLiftLimit = hardwareMap.get(DigitalChannel.class, "right_lift_limit0");
        blockSensor = hardwareMap.get(DistanceSensor.class, "left_block_sensor");
        liftSensor = hardwareMap.get(DistanceSensor.class, "left_height_sensor");

        //leftExtensionLimit = hardwareMap.get(TouchSensor.class, "leftExtensionLimit");
        rightExtensionLimit = hardwareMap.get(DigitalChannel.class, "right_extension_limit0");

        front_left_wheel.setDirection(DcMotor.Direction.REVERSE);
        back_left_wheel.setDirection(DcMotor.Direction.REVERSE);
        front_right_wheel.setDirection(DcMotor.Direction.FORWARD);
        back_right_wheel.setDirection(DcMotor.Direction.FORWARD);

        front_left_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftExtensionServo.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLiftServo.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLiftServo.setDirection(DcMotorSimple.Direction.FORWARD);

        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);


        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);

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


        period.reset();

        while (opModeIsActive()) {

            drive();

            resetAngle();

            //Pressing gamepad2.x is for closing and opening claw
            claw();

            //gamepad2 right bumper is for extending and right trigger for retracting
            extend();

            telemetry.addLine("");

            //gamepad2 left bumper is for lift going up and left trigger for going down.
            lift();

            //gamepad1 left trigger for intake and right trigger for outtake
            intake();

            getHeading();

            telemetry.addLine("");

            if (isBlockIn()) {
                telemetry.addLine("BLOCK IS IN");
            }

            telemetry.addLine("");

            telemetry.addData("Height of lift ", liftSensor.getDistance(DistanceUnit.INCH));

            if (Xpressed == false){
                telemetry.addLine("CLAW IS CLOSED");
            }

            telemetry.addLine("");

            telemetry.update();
            telemetry.clear();
        }
    }


    public void drive() {

        if (gamepad1.x && !X1StillPressed) {
            if (!X1pressed) {
                turnFactor = 3;
                driveFactor = 2;
                X1pressed = true;
            } else {
                turnFactor = 2;
                driveFactor = 1;
                X1pressed = false;
            }
            X1StillPressed = true;
        }
        if (!gamepad1.x) {
            X1StillPressed = false;
        }

        double scaleFactor = 1;
        double Protate = gamepad1.right_stick_x / turnFactor;
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

        //Robot Centric
        //gyroAngle = -Math.PI / 2;

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



        front_left_wheel.setPower(((Py - Protate) / scaleFactor)/driveFactor);
        back_left_wheel.setPower(((Px - Protate) / scaleFactor)/driveFactor);
        back_right_wheel.setPower(((Py + Protate) / scaleFactor)/driveFactor);
        front_right_wheel.setPower(((Px + Protate) / scaleFactor)/driveFactor);

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
            }
            else {
                clawServo.setPower(.7);
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

        if (Math.abs(gamepad2.right_trigger) > 0.05 && !rightExtensionLimit.getState()) {
            extensionPower = -0.4;
        }
        else if (gamepad2.right_bumper) {
            extensionPower = 0.7;
        }
        else if (!rightExtensionLimit.getState()) {
            extensionPower = .05;
        }
        else{
            extensionPower = 0;
        }
        leftExtensionServo.setPower(extensionPower);

    }

    public void lift() {
        if (gamepad2.left_bumper) {
            liftPower = 0.7;
        }
        else if ((Math.abs(gamepad2.left_trigger) > .07) && !rightLiftLimit.getState()) {
            liftPower = -0.1;

        }
        else if (!rightLiftLimit.getState()) {
            liftPower = 0.15;
            telemetry.addLine("LIFT IS UP");
        }
        else {
            liftPower = 0;
        }

        leftLiftServo.setPower(liftPower);
        rightLiftServo.setPower(liftPower);

    }

    public void intake() {
        if (gamepad1.left_trigger > .05) {
            rightIntake.setPower(1);
            leftIntake.setPower(.7);
            intake2.setPower(1);
            intake3.setPower(1);
            telemetry.addLine("INTAKING");
        }
        else if (gamepad1.right_trigger > .05) {
            rightIntake.setPower(-1);
            leftIntake.setPower(-1);
            intake2.setPower(-1);
            intake3.setPower(-1);
            telemetry.addLine("OUTTAKING");

        }
        else {
            rightIntake.setPower(0);
            leftIntake.setPower(0);
            intake2.setPower(0);
            intake3.setPower(0);
        }
    }

    public boolean isBlockIn(){
        if (blockSensor.getDistance(DistanceUnit.INCH)<2){
            return true;
        }
        return  false;
    }
}

