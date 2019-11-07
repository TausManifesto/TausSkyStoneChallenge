package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Disabled
@TeleOp(name = "Teleop-A", group = "Tau")
public class TeleOp_A extends LinearOpMode {

    float rotate_angle = 0;
    double reset_angle = 0;
    private CRServo clawServo = null;

    private DcMotor front_left_wheel = null;
    private DcMotor back_left_wheel = null;
    private DcMotor back_right_wheel = null;
    private DcMotor front_right_wheel = null;
    private double leftLiftPower = 0;
    private double rightLiftPower = 0;
    private double leftExtensionPower = 0;
    private double rightExtensionPower = 0;


    public CRServo leftLiftServo = null;
    public CRServo rightLiftServo = null;
    public CRServo leftExtensionServo = null;
    public CRServo rightExtensionServo = null;

    BNO055IMU imu;
    @Override
    public void runOpMode() {
        Hardware robot = new Hardware();

        front_left_wheel = hardwareMap.dcMotor.get("left_front");
        back_left_wheel = hardwareMap.dcMotor.get("left_back");
        back_right_wheel = hardwareMap.dcMotor.get("right_back");
        front_right_wheel = hardwareMap.dcMotor.get("right_front");
        leftLiftServo = hardwareMap.crservo.get("left_lift");
        rightLiftServo = hardwareMap.crservo.get("right_lift");
        leftExtensionServo = hardwareMap.crservo.get("left_extension");
        rightExtensionServo = hardwareMap.crservo.get("right_extension");

        front_left_wheel.setDirection(DcMotor.Direction.REVERSE);
        back_left_wheel.setDirection(DcMotor.Direction.REVERSE);
        front_right_wheel.setDirection(DcMotor.Direction.FORWARD);
        back_right_wheel.setDirection(DcMotor.Direction.FORWARD);

        front_left_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu.initialize(parameters);

        clawServo = hardwareMap.crservo.get("intake_servo");
        clawServo.setPower(0);

        while(!opModeIsActive()){}

        while(opModeIsActive()){
            drive();
            resetAngle();
             if (gamepad1.y)
                 driveSimple();
            if(gamepad2.x) {
                telemetry.addData("Grabbing block", gamepad2.x);
                telemetry.update();
                grabBlock("grab");
            }

            if(gamepad2.b) {
                telemetry.addData("Releasing block", gamepad2.b);
                telemetry.update();
                grabBlock("release");
            }



            if(Math.abs(gamepad2.left_stick_x) > 0.05){
                telemetry.addData("Extension", gamepad2.left_stick_x);
                telemetry.update();

                leftExtensionPower = gamepad2.left_stick_x;
                rightExtensionPower = -gamepad2.left_stick_x;
                leftExtensionServo.setPower(leftExtensionPower);
                rightExtensionServo.setPower(rightExtensionPower);
                sleep(500);
                leftExtensionServo.setPower(0);
                rightExtensionServo.setPower(0);

            }
            if(Math.abs(gamepad2.right_stick_y) > 0.05){
                telemetry.addData("Lift", gamepad2.left_stick_y);
                telemetry.update();

                leftLiftPower = gamepad2.right_stick_y;
                rightLiftPower = -gamepad2.right_stick_y;
                leftLiftServo.setPower(leftLiftPower);
                rightLiftServo.setPower(rightLiftPower);
                sleep(500);
                leftLiftServo.setPower(0);
                rightLiftServo.setPower(0);

            }
            telemetry.update();

        }
    }


    public void driveSimple(){
        double power = .6;
        if(gamepad1.dpad_up){ //Forward
            front_left_wheel.setPower(-power);
            back_left_wheel.setPower(-power);
            back_right_wheel.setPower(-power);
            front_right_wheel.setPower(-power);
        }
        else if(Math.abs(gamepad1.right_stick_x) > 0){ //Rotation
            front_left_wheel.setPower(-gamepad1.right_stick_x);
            back_left_wheel.setPower(-gamepad1.right_stick_x);
            back_right_wheel.setPower(gamepad1.right_stick_x);
            front_right_wheel.setPower(gamepad1.right_stick_x);
        }
        else{
            front_left_wheel.setPower(0);
            back_left_wheel.setPower(0);
            back_right_wheel.setPower(0);
            front_right_wheel.setPower(0);
        }
    }
    public void drive() {
        double Protate = gamepad1.right_stick_x/4;
        double stick_x = gamepad1.right_stick_x * Math.sqrt(Math.pow(1-Math.abs(Protate), 2)/2); //Accounts for Protate when limiting magnitude to be less than 1
        double stick_y = gamepad1.right_stick_y * Math.sqrt(Math.pow(1-Math.abs(Protate), 2)/2);
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

        if(gamepad1.right_bumper){ //Disables gyro, sets to -Math.PI/2 so front is defined correctly.
            gyroAngle = -Math.PI/2;
        }

        //Linear directions in case you want to do straight lines.
        if(gamepad1.dpad_right){
            stick_x = 0.7;
        }
        else if(gamepad1.dpad_left){
            stick_x = -0.7;
        }
        if(gamepad1.dpad_up){
            stick_y = -0.7;
        }
        else if(gamepad1.dpad_down){
            stick_y = 0.7;
        }



        //MOVEMENT
        theta = Math.atan2(stick_y, stick_x) - gyroAngle - (Math.PI / 2);
        Px = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * (Math.sin(theta + Math.PI / 4));
        Py = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * (Math.sin(theta - Math.PI / 4));

        telemetry.addData("Stick_X", stick_x);
        telemetry.addData("Stick_Y", stick_y);
        telemetry.addData("Magnitude",  Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)));
        telemetry.addData("Front Left", Py - Protate);
        telemetry.addData("Back Left", Px - Protate);
        telemetry.addData("Back Right", Py + Protate);
        telemetry.addData("Front Right", Px + Protate);

        front_left_wheel.setPower(Py - Protate);
        back_left_wheel.setPower(Px - Protate);
        back_right_wheel.setPower(Py + Protate);
        front_right_wheel.setPower(Px + Protate);
    }
    public void resetAngle(){
        if(gamepad1.a){
            reset_angle = getHeading() + reset_angle;
        }
    }
    public double getHeading(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;
        if(heading < -180)
            heading = heading + 360;
        else if(heading > 180){
            heading = heading - 360;
        }
        heading = heading - reset_angle;
        return heading;
    }



    public void grabBlock (String type) {


        if (type.equalsIgnoreCase("grab")) {

            clawServo.setPower(0.3);
            sleep(500);
            clawServo.setPower(0);


        } else if (type.equalsIgnoreCase("release")) {

            clawServo.setPower(-0.3);
            sleep(500);
            clawServo.setPower(0);


        }


    }

}