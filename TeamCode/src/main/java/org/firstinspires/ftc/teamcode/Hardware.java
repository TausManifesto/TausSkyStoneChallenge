package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by Lance He 9/16/2017.
 */


public class Hardware {

    // Motor variable names
    public DcMotor frontLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor backLeftMotor = null;
    public DcMotor backRightMotor = null;
    public DcMotor leftIntake = null;
    public DcMotor rightIntake = null;
    public CRServo intake2 = null;
    public CRServo intake3 = null;
    public CRServo clawServo = null;
    public CRServo leftLiftServo = null;
    public CRServo rightLiftServo = null;
    public CRServo leftExtensionServo = null;
    public DigitalChannel rightLimitSwitch;
    public DigitalChannel leftLimitSwitch;
    public DistanceSensor blockSensor = null;

    // Other variable names
    HardwareMap hwMap;
    private ElapsedTime period = new ElapsedTime();


    public Hardware() {
        hwMap = null;
    }


    public void init(HardwareMap hwMap) {

        // Save reference to Hardware map
        this.hwMap = hwMap;
        period.reset();

        // Define Motors
        frontLeftMotor = hwMap.dcMotor.get("left_front");
        frontRightMotor = hwMap.dcMotor.get("right_front");
        backLeftMotor = hwMap.dcMotor.get("left_back");
        backRightMotor = hwMap.dcMotor.get("right_back");

        //frontLeftMotor = hwMap.dcMotor.get("front_left");
        //frontRightMotor = hwMap.dcMotor.get("front_right");
        //backLeftMotor = hwMap.dcMotor.get("back_left");
        //backRightMotor = hwMap.dcMotor.get("back_right");

        // Initialize Motors
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ******MAY CHANGE *******  Fix Forward/Reverse under testing

        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);


    }


    public void initTeleOpDelayIMU(HardwareMap hwMap) {

        // Save reference to Hardware map
        this.hwMap = hwMap;
        period.reset();

        // Define Motors
        frontLeftMotor = hwMap.dcMotor.get("left_front");
        frontRightMotor = hwMap.dcMotor.get("right_front");
        backLeftMotor = hwMap.dcMotor.get("left_back");
        backRightMotor = hwMap.dcMotor.get("right_back");

        // Initialize Motors
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ******MAY CHANGE *******  Fix Forward/Reverse under testing
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        // May use RUN_USING_ENCODERS if encoders are installed
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define Servos
        leftLiftServo = hwMap.crservo.get("left_lift");
        rightLiftServo = hwMap.crservo.get("right_lift");

        leftLiftServo.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLiftServo.setDirection(DcMotorSimple.Direction.FORWARD);

        leftLiftServo.setPower(0);
        rightLiftServo.setPower(0);

    }


    public void initTeleOpNOIMU(HardwareMap hwMap) {

        // Save reference to Hardware map
        this.hwMap = hwMap;
        period.reset();

        // Define Motors
        frontLeftMotor = hwMap.dcMotor.get("left_front");
        frontRightMotor = hwMap.dcMotor.get("right_front");
        backLeftMotor = hwMap.dcMotor.get("left_back");
        backRightMotor = hwMap.dcMotor.get("right_back");
        rightIntake = hwMap.dcMotor.get("right_intake");
        leftIntake = hwMap.dcMotor.get("left_intake");
        intake2 = hwMap.crservo.get("intake_help_right");
        intake3 = hwMap.crservo.get("intake_help_left");
        blockSensor = hwMap.get(DistanceSensor.class, "left_block_sensor");


        // Initialize Motors
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ******MAY CHANGE *******  Fix Forward/Reverse under testing
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        leftIntake.setPower(0);
        rightIntake.setPower(0);

        // May use RUN_USING_ENCODERS if encoders are installed
        //frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Define Servos
        leftLiftServo = hwMap.crservo.get("left_lift");
        rightLiftServo = hwMap.crservo.get("right_lift");
        leftExtensionServo = hwMap.crservo.get("left_extension");
        clawServo = hwMap.crservo.get("claw_servo");

        leftLiftServo.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLiftServo.setDirection(DcMotorSimple.Direction.REVERSE);
        leftExtensionServo.setDirection(DcMotorSimple.Direction.FORWARD);

        leftLiftServo.setPower(0);
        rightLiftServo.setPower(0);
        leftExtensionServo.setPower(0);


        //Define Limit Switches
        rightLimitSwitch = hwMap.get(DigitalChannel.class, "right_lift_limit0");
        leftLimitSwitch = hwMap.get(DigitalChannel.class, "left_lift_limit0");




    }


    public double getTime(){

        return period.time();

    }


}