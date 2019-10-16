package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


/**

 * created by Ashwin Jandhyala

 */

@Autonomous(name = "2019_FTC_Auto", group="Tau")

public class Auto_Mechanum extends LinearOpMode {
    BNO055IMU imu;
    //@Override
    ElapsedTime runTime = new ElapsedTime();

    double degreesCalc;


    double heading;
    double initAngle;
    double xVelocity;
    double yVelocity;
    double velocity;
    double distTraveled;
    double distTraveledX;
    double distTraveledY;
    double direcTraveled;

    double initialX = 0;
    double initialY = 0;

    double distX;
    double distY;
    double currentX = initialX;
    double currentY = initialY;
    int cycles = 0;
    double allVelocities;
    double avgVelocity;


    Hardware robot = new Hardware();



    //Defining methods for getting imu reading

    public double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = angles.firstAngle;
        return heading;//right :: [0, -180] -- left :: [0, 180]
    }

    public double getVelocityX() {
        Velocity velocity = imu.getVelocity();
        xVelocity = velocity.xVeloc;
        return xVelocity;
    }

    public double getVelocityY() {
        Velocity velocity = imu.getVelocity();
        yVelocity = velocity.yVeloc;
        return yVelocity;

    }

    public double getVelocity() {
        velocity = Math.sqrt(Math.pow(getVelocityX(), 2) + Math.pow(getVelocityY(), 2));
        return velocity;
    }

    public double getAvgVelocity() {
        allVelocities += getVelocity();
        avgVelocity += allVelocities / cycles;
        cycles++;
        return avgVelocity;
    }

    public double getDistanceTraveled(double time) {
        distTraveled = getAvgVelocity() * time;
        return distTraveled;
    }

    public double getDistanceTraveledX(double time) {
        distTraveledX = getVelocityX() * time;
        return distTraveledX;
    }

    public double getDistanceTraveledY(double time) {
        distTraveledY = getVelocityY() * time;
        return distTraveledY;
    }

    public double getdirectionTraveled() {
        direcTraveled = Math.atan(getVelocityY() / getVelocityX());
        return direcTraveled;
    }


    //Defining Methods for the robot

    public void forward(double power, double distance) {
        robot.backLeftMotor.setPower(power);

        robot.backRightMotor.setPower(-power);//+

        robot.frontRightMotor.setPower(-power);//+

        robot.frontLeftMotor.setPower(power);

        runTime.reset();

        initAngle = getHeading();

        while (distance < getDistanceTraveled(runTime.seconds())) {

            if (getHeading() > initAngle + 10) {
                right(.1, getHeading() - initAngle);
            }

            else if (getHeading() < initAngle - 10) {
                left(.1, initAngle - getHeading());
            }

        }


        robot.backLeftMotor.setPower(0);

        robot.backRightMotor.setPower(0);

        robot.frontRightMotor.setPower(0);

        robot.frontLeftMotor.setPower(0);

    }

    public void backward(double power, long distance) {
        robot.backLeftMotor.setPower(-power);

        robot.backRightMotor.setPower(power);//-

        robot.frontRightMotor.setPower(power);//-

        robot.frontLeftMotor.setPower(-power);

        runTime.reset();

        initAngle = getHeading();

        while (distance < getDistanceTraveled(runTime.seconds())) {

            if (getHeading() > initAngle + 10) {
                right(.1, getHeading() - initAngle);
            }

            else if (getHeading() < initAngle - 10) {
                left(.1, initAngle - getHeading());
            }

        }

        robot.backLeftMotor.setPower(0);

        robot.backRightMotor.setPower(0);

        robot.frontRightMotor.setPower(0);

        robot.frontLeftMotor.setPower(0);

        currentX += distX;
        currentY += distY;

    }

    public void right(double power, double degrees) {

        robot.backLeftMotor.setPower(power);

        robot.backRightMotor.setPower(power);//-

        robot.frontRightMotor.setPower(power);//-

        robot.frontLeftMotor.setPower(power);

        initAngle = getHeading();

        while (initAngle - getHeading() < degrees) {
            telemetry.addData("deg :: ", getHeading() - initAngle);
            telemetry.update();

        }

        robot.backLeftMotor.setPower(0);

        robot.backRightMotor.setPower(0);

        robot.frontRightMotor.setPower(0);

        robot.frontLeftMotor.setPower(0);

    }

    public void left(double power, double degrees) {

        robot.backLeftMotor.setPower(-power);

        robot.backRightMotor.setPower(-power);//+

        robot.frontRightMotor.setPower(-power);//+

        robot.frontLeftMotor.setPower(-power);

        initAngle = getHeading();

        while (getHeading() - initAngle < degrees) {

        }
        robot.backLeftMotor.setPower(0);

        robot.backRightMotor.setPower(0);

        robot.frontRightMotor.setPower(0);

        robot.frontLeftMotor.setPower(0);

    }

    public void strafeLeft(double power, long distance) {
        robot.backLeftMotor.setPower(power);

        robot.backRightMotor.setPower(power);//-

        robot.frontRightMotor.setPower(-power);//+

        robot.frontLeftMotor.setPower(-power);

        runTime.reset();

        initAngle = getHeading();

        while (distance < getDistanceTraveled(runTime.seconds())) {

            if (getHeading() > initAngle + 10) {
                right(.1, getHeading() - initAngle);
            }

            else if (getHeading() < initAngle - 10) {
                left(.1, initAngle - getHeading());
            }

        }

        robot.backLeftMotor.setPower(0);

        robot.backRightMotor.setPower(0);

        robot.frontRightMotor.setPower(0);

        robot.frontLeftMotor.setPower(0);

    }

    public void strafeRight(double power, long distance) {
        robot.backLeftMotor.setPower(-power);

        robot.backRightMotor.setPower(-power);//+

        robot.frontRightMotor.setPower(power);//-

        robot.frontLeftMotor.setPower(power);

        runTime.reset();

        initAngle = getHeading();

        while (distance < getDistanceTraveled(runTime.seconds())) {

            if (getHeading() > initAngle + 10) {
                right(.1, getHeading() - initAngle);
            }

            else if (getHeading() < initAngle - 10) {
                left(.1, initAngle - getHeading());
            }

        }

        robot.backLeftMotor.setPower(0);

        robot.backRightMotor.setPower(0);

        robot.frontRightMotor.setPower(0);

        robot.frontLeftMotor.setPower(0);

    }

    public void direction(double power, long distance, double degrees) {
        //rotating degrees right by 45 in order to grt angle relative to back left wheel
        degreesCalc = degrees - 45;

        //calculating necessary magnitude of the x vector based on angle
        double xVector = power * Math.cos(degreesCalc);

        //calculating the necessary magnitude of the y vector based on angle
        double yVector = power * Math.sin(degreesCalc);

        //the back left motor is on the x axis
        //assign it the x vector
        robot.backLeftMotor.setPower(xVector);

        //the back left motor is on the x axis
        //assign it the y vector
        robot.backRightMotor.setPower(-yVector);//+

        //the back left motor is on the x axis
        //assign it the x vector
        robot.frontRightMotor.setPower(-xVector);//+

        //the back left motor is on the x axis
        //assign it the y vector
        robot.frontLeftMotor.setPower(yVector);

        runTime.reset();

        initAngle = getHeading();

        while (distance < getDistanceTraveled(runTime.seconds())) {

            if (getHeading() > initAngle + 10) {
                right(.1, getHeading() - initAngle);
            }

            else if (getHeading() < initAngle - 10) {
                left(.1, initAngle - getHeading());
            }

        }

        robot.backLeftMotor.setPower(0);

        robot.backRightMotor.setPower(0);

        robot.frontRightMotor.setPower(0);

        robot.frontLeftMotor.setPower(0);

    }


    @Override
    public void runOpMode() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);

        robot.initTeleOpNOIMU(hardwareMap);
        waitForStart();

        //forward(1, 1);

        //backward(.1, 10000);

        //strafeRight(.1, 10000);

        //strafeLeft(.1, 10000};

        //right(.2, 90);

        //left(.2, 90);

        //direction(1, 1000, 50);

        /*while (true) {
            telemetry.addData();
            telemetry.update();
        }*/

    }
}