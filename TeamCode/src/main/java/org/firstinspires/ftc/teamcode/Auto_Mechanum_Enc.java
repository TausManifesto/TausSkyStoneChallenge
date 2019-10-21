//code for the autonomous portion of the Skystone challenge

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**

 * created by ashwin jandhyala
 * 10/6/2019

 */
//@Disabled
@Autonomous(name = "2019_FTC_Auto_Enc", group="Tau")

public class Auto_Mechanum_Enc extends LinearOpMode {

    private BNO055IMU imu;
    //@Override
    private ElapsedTime runTime = new ElapsedTime();

    private Hardware robot = new Hardware();

    //defining methods for returning imu reading

    //getting angle based on initial angle
    private double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle; //right :: [0, -180] -- left :: [0, 180]
    }

    private  double sigmoid(double error){

        if (error>40){
            return .2;
        }

        else {
            return .11;
        }
        //return  1/(1 + Math.pow(3, -error/9))-.5;
    }


    //defining Methods for the robot

    //moving forward distance (m) with power [0, 1]
    private void forward(double power, double distance) {

        int counts = (int)((distance/(4*Math.PI))*1075);
        robot.backLeftMotor.setTargetPosition(counts);
        robot.backRightMotor.setTargetPosition(counts);
        robot.frontRightMotor.setTargetPosition(counts);
        robot.frontLeftMotor.setTargetPosition(counts);

        //setting all motors to go forward (positive)

        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftMotor.setPower(power);
        robot.backRightMotor.setPower(power);
        robot.frontRightMotor.setPower(power);
        robot.frontLeftMotor.setPower(power);

        while (opModeIsActive() && robot.backLeftMotor.isBusy() && robot.backRightMotor.isBusy() && robot.frontRightMotor.isBusy() && robot.frontLeftMotor.isBusy()){
        }


        //setting all motor powers to 0 (stopping)
        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.backRightMotor.setPower(0);
        //robot.frontRightMotor.setPower(0);
        //robot.frontLeftMotor.setPower(0);
        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //moving backward distance (m) with power [0, 1]
    private void backward(double power, long distance) {

        int counts = (int)((distance/(4*Math.PI))*1075);
        robot.backLeftMotor.setTargetPosition(-counts);
        robot.backRightMotor.setTargetPosition(-counts);
        robot.frontRightMotor.setTargetPosition(-counts);
        robot.frontLeftMotor.setTargetPosition(-counts);

        //setting all motors to go forward (positive)

        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftMotor.setPower(power);
        robot.backRightMotor.setPower(power);
        robot.frontRightMotor.setPower(power);
        robot.frontLeftMotor.setPower(power);

        while (opModeIsActive() && robot.backLeftMotor.isBusy() && robot.backRightMotor.isBusy() && robot.frontRightMotor.isBusy() && robot.frontLeftMotor.isBusy()){
        }


        //setting all motor powers to 0 (stopping)
        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.backRightMotor.setPower(0);
        //robot.frontRightMotor.setPower(0);
        //robot.frontLeftMotor.setPower(0);
        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //turning right angle (deg) with power [0, 1]
    private void right(double degrees) {

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double initAngle = getHeading(); //angle that the robot is at when it starts

        //wait until angle turned is >= angle inputted
            while (initAngle - getHeading() <= degrees) {

            //setting left motors to go forward (positive power)
            robot.backLeftMotor.setPower(sigmoid(degrees-(initAngle-getHeading())));
            robot.frontLeftMotor.setPower(sigmoid(degrees-(initAngle-getHeading())));

            //setting right motors to go backward (negative power)
            robot.backRightMotor.setPower(-sigmoid(degrees-(initAngle-getHeading()))); //-
            robot.frontRightMotor.setPower(-sigmoid(degrees-(initAngle-getHeading()))); //-
        }

        //setting motor value to 0 (stop)
        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("degrees turned :: ", initAngle - getHeading());
        telemetry.update();

    }

    //turning left angle (deg) with power [0, 1]
    private void left(double degrees) {

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double initAngle = getHeading(); //angle that the robot is at when it starts

        //wait until angle turned is >= angle inputted
        while (getHeading()-initAngle <= degrees) {

            //setting left motors to go forward (positive power)
            robot.backLeftMotor.setPower(-sigmoid(degrees-(initAngle-getHeading())));
            robot.frontLeftMotor.setPower(-sigmoid(degrees-(initAngle-getHeading())));

            //setting right motors to go backward (negative power)
            robot.backRightMotor.setPower(sigmoid(degrees-(initAngle-getHeading()))); //-
            robot.frontRightMotor.setPower(sigmoid(degrees-(initAngle-getHeading()))); //-
        }

        //setting motor value to 0 (stop)
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.frontLeftMotor.setPower(0);

        telemetry.addData("degrees turned :: ", initAngle - getHeading());
        telemetry.update();

    }

    //strafing left distance (m) with power [0, 1]
    private void strafeLeft(double power, long distance) {
        int counts = (int)((distance/(4*Math.PI))*1075);
        robot.backLeftMotor.setTargetPosition(counts);
        robot.backRightMotor.setTargetPosition(-counts);
        robot.frontRightMotor.setTargetPosition(counts);
        robot.frontLeftMotor.setTargetPosition(-counts);

        //setting all motors to go forward (positive)

        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftMotor.setPower(power);
        robot.backRightMotor.setPower(-power);
        robot.frontRightMotor.setPower(power);
        robot.frontLeftMotor.setPower(-power);

        while (opModeIsActive() && robot.backLeftMotor.isBusy() && robot.backRightMotor.isBusy() && robot.frontRightMotor.isBusy() && robot.frontLeftMotor.isBusy()){
        }


        //setting all motor powers to 0 (stopping)
        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.backRightMotor.setPower(0);
        //robot.frontRightMotor.setPower(0);
        //robot.frontLeftMotor.setPower(0);
        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

    //strafing right distance (m) with power [0, 1]
    private void strafeRight(double power, long distance) {

        int counts = (int)((distance/(4*Math.PI))*1075);
        robot.backLeftMotor.setTargetPosition(-counts);
        robot.backRightMotor.setTargetPosition(counts);
        robot.frontRightMotor.setTargetPosition(-counts);
        robot.frontLeftMotor.setTargetPosition(counts);

        //setting all motors to go forward (positive)

        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftMotor.setPower(-power);
        robot.backRightMotor.setPower(power);
        robot.frontRightMotor.setPower(-power);
        robot.frontLeftMotor.setPower(power);

        while (opModeIsActive() && robot.backLeftMotor.isBusy() && robot.backRightMotor.isBusy() && robot.frontRightMotor.isBusy() && robot.frontLeftMotor.isBusy()){
        }


        //setting all motor powers to 0 (stopping)
        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.backRightMotor.setPower(0);
        //robot.frontRightMotor.setPower(0);
        //robot.frontLeftMotor.setPower(0);
        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }


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

        waitForStart();

        //after start is pressed

        //runTime.reset();

        //red -- foundation
        forward(.4, 30);
        sleep(1000);
        backward(.4, 22);
        sleep(1000);
        strafeLeft(.4, 50);
        sleep(1000);

        //red -- blocks

        //blue -- foundation
        forward(.4, 30);
        sleep(1000);
        backward(.4, 22);
        sleep(1000);
        strafeRight(.4, 50);
        sleep(1000);

        //blue -- blocks


    }
}