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

    float rotate_angle = 0;
    double reset_angle = 0;
    double intakePower = 0;
    double clawPower = 0;
    boolean clawIsOpen = false;


    private DcMotor front_left_wheel = null;
    private DcMotor back_left_wheel = null;
    private DcMotor back_right_wheel = null;
    private DcMotor front_right_wheel = null;

    BNO055IMU imu;

    private double liftPower = 0;
    private double extensionPower = 0;

    private CRServo clawServo = null;
    public CRServo leftLiftServo = null;
    public CRServo rightLiftServo = null;
    public CRServo leftExtensionServo = null;
    public CRServo rightExtensionServo = null;

    private TouchSensor leftLiftLimit = null;
    private TouchSensor rightLiftLimit = null;
    //private TouchSensor leftExtensionLimit = null;
    //private TouchSensor rightExtensionLimit = null;

    private DcMotor right_intake = null;
    private DcMotor left_intake = null;



    @Override
    public void runOpMode() {
        front_left_wheel = hardwareMap.dcMotor.get("left_front");
        back_left_wheel = hardwareMap.dcMotor.get("left_back");
        back_right_wheel = hardwareMap.dcMotor.get("right_back");
        front_right_wheel = hardwareMap.dcMotor.get("right_front");
        leftLiftServo = hardwareMap.crservo.get("left_lift");
        rightLiftServo = hardwareMap.crservo.get("right_lift");
        leftExtensionServo = hardwareMap.crservo.get("left_extension");
        rightExtensionServo = hardwareMap.crservo.get("right_extension");
        clawServo = hardwareMap.crservo.get("claw_servo");
        leftLiftLimit = hardwareMap.get(TouchSensor.class, "left_lift_limit0");
        rightLiftLimit = hardwareMap.get(TouchSensor.class, "right_lift_limit0");
        //leftExtensionLimit = hardwareMap.get(TouchSensor.class, "leftExtensionLimit");
        //rightExtensionLimit = hardwareMap.get(TouchSensor.class, "rightExtensionLimit");
        right_intake = hardwareMap.dcMotor.get("right_intake");
        left_intake = hardwareMap.dcMotor.get("left_intake");


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
        rightLiftServo.setDirection(DcMotorSimple.Direction.REVERSE);

        right_intake.setDirection(DcMotorSimple.Direction.FORWARD);
        left_intake.setDirection(DcMotorSimple.Direction.REVERSE);



        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);

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

            //gamepad1.y is for the block to be intaked
            intake();

            //gamepad1.b is for the block to be outtaked
            outtake();

            //gamepad1.x is for the intake to stop
            stopIntake();

            telemetry.update();

        }
    }



    public void drive() {
        double Protate =  gamepad1.right_stick_x/ 3;
        double stick_x =  gamepad1.left_stick_x * Math.sqrt(Math.pow(1 - Math.abs(Protate), 2) / 2); //Accounts for Protate when limiting magnitude to be less than 1
        double stick_y =  gamepad1.left_stick_y * Math.sqrt(Math.pow(1 - Math.abs(Protate), 2) / 2);
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
            stick_x = 1.0;
        } else if (gamepad1.dpad_left) {
            stick_x = -1.0;
        }
        if (gamepad1.dpad_up) {
            stick_y = -1.0;
        } else if (gamepad1.dpad_down) {
            stick_y = 1.0;
        }


        //MOVEMENT for rotation
        theta = Math.atan2(stick_y, stick_x) - gyroAngle - (Math.PI / 2);
        Px = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * (Math.sin(theta + Math.PI / 4));
        Py = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * (Math.sin(theta - Math.PI / 4));

        telemetry.addData("Stick_X", stick_x);
        telemetry.addData("Stick_Y", stick_y);
        telemetry.addData("Magnitude", Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)));
        telemetry.addData("Front Left", Py - Protate);
        telemetry.addData("Back Left", Px - Protate);
        telemetry.addData("Back Right", Py + Protate);
        telemetry.addData("Front Right", Px + Protate);

        front_left_wheel.setPower(Py - Protate);
        back_left_wheel.setPower(Px - Protate);
        back_right_wheel.setPower(Py + Protate);
        front_right_wheel.setPower(Px + Protate);
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

        if(gamepad2.x){
            clawServo.setPower(0.2);

        } else if(gamepad2.b) {
            clawServo.setPower(-0.7);
            sleep(500);
            clawServo.setPower(0.3);

        }else{

            clawServo.setPower(0);
        }

    }

    public void extend(){

        if (Math.abs(gamepad2.right_trigger) > 0.05) {
            telemetry.addData("extension", gamepad2.right_trigger);
            extensionPower = 0.4;
        }
        else if(gamepad2.right_bumper){
            extensionPower = -0.4;
        }
        else{
            extensionPower = 0.0;
        }
        leftExtensionServo.setPower(extensionPower);
        rightExtensionServo.setPower(extensionPower);

    }

    public void lift(){
        if (Math.abs(gamepad2.left_trigger) > 0.07 ) {
            telemetry.addData("lift", gamepad2.left_trigger);
            liftPower = 0.5;
            sleep(500);
            liftPower = 0.1;
        }else if(gamepad2.left_bumper){
            liftPower = -0.1;

        } else {
            liftPower = 0;
        }

        leftLiftServo.setPower(liftPower);
        rightLiftServo.setPower(liftPower);

    }

    public void intake() {

        if (gamepad1.a) {
            intakePower = 0.7;
        }
        telemetry.addData("intakePower", intakePower);
        telemetry.update();
        right_intake.setPower(intakePower);
        left_intake.setPower(intakePower);

    }

    public void outtake() {

        if (gamepad1.b) {
            intakePower = -0.7;
        }

        telemetry.addData("outtakePower", intakePower);
        telemetry.update();
        right_intake.setPower(intakePower);
        left_intake.setPower(intakePower);
    }

    public void stopIntake() {

        if (gamepad1.x) {

            intakePower = 0;
        }

        right_intake.setPower(intakePower);
        left_intake.setPower(intakePower);
    }
}