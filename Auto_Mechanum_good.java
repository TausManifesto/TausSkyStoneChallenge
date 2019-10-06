//code for the autonomous portion of the Skystone challenge

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

 * created by ashwin jandhyala
 * 10/6/2019

 */

@Autonomous(name = "2019_FTC_Auto", group="Tau")

public class Auto_Mechanum_good extends LinearOpMode {

    private BNO055IMU imu;
    //@Override
    private ElapsedTime runTime = new ElapsedTime();

    //initializing variables for getting average velocity
    private int cycles = 0;
    private double allVelocities;
    private double avgVelocity;


    private Hardware robot = new Hardware();


    //defining methods for returning imu reading

    //getting angle based on initial angle
    private double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle; //right :: [0, -180] -- left :: [0, 180]
    }

    //getting x velocity (m/s)
    private double getVelocityX() {
        Velocity velocity = imu.getVelocity();
        return velocity.xVeloc;
    }

    //getting y velocity (m/s)
    private double getVelocityY() {
        Velocity velocity = imu.getVelocity();
        return velocity.yVeloc;
    }

    //getting absolute velocity (m/s)
    private double getVelocity() {
        //using pythagorean's theorem to calculate velocity using x and y vectors
        //velocity = root(x^2 + y^2)
        return Math.sqrt(Math.pow(getVelocityX(), 2) + Math.pow(getVelocityY(), 2));
    }

    //getting the average velocity (m/s)
    private double getAvgVelocity(boolean reset) {
        if (reset) {
            cycles = 0;
            allVelocities = 0;
            avgVelocity = 0;
        }
        else {
            //averaging out all velocities
            cycles++; //how many velocities we are averaging
            allVelocities += getVelocity(); //adding instantaneous velocity
            avgVelocity += allVelocities / cycles; //averaging velocities by dividing by number of velocities
        }
        return avgVelocity;
    }

    //getting distance traveled (m)
    private double getDistanceTraveled(double time) {
        //calculating distance using average velocity
        //distance = change in velocity * change in time
        return getAvgVelocity(false) * time;
    }

    //getting direction travelled
    private double getDirectionTraveled() {
        //the reverse tangent of the y/x velocity
        return Math.atan(getVelocityY() / getVelocityX());
    }


    //defining Methods for the robot

    //moving forward distance (m) with power [0, 1]
    private void forward(double power, double distance) {

        //setting all motors to go forward (positive)
        robot.backLeftMotor.setPower(power);
        robot.backRightMotor.setPower(-power);//+
        robot.frontRightMotor.setPower(-power);//+
        robot.frontLeftMotor.setPower(power);

        runTime.reset(); //staring stopwatch

        double initAngle = getHeading(); //getting starting angle

        //waiting until distance traveled >= distance inputted
        while (distance >= getDistanceTraveled(runTime.seconds())) {
            //checking current angle against starting angle and adjusting for any offset greater than 10 deg
            if (getHeading() > initAngle + 10) {
                right(.1, getHeading() - initAngle); //turning right how much we are offset
            }
            else if (getHeading() < initAngle - 10) {
                left(.1, initAngle - getHeading());
            }
        }

        //setting all motor powers to 0 (stopping)
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.frontLeftMotor.setPower(0);

        getAvgVelocity(true); //resetting the average velocity
    }

    //moving backward distance (m) with power [0, 1]
    private void backward(double power, long distance) {

        //setting all motors to go backward (-power)
        robot.backLeftMotor.setPower(-power);
        robot.backRightMotor.setPower(power);//-
        robot.frontRightMotor.setPower(power);//-
        robot.frontLeftMotor.setPower(-power);

        runTime.reset(); //staring stopwatch

        double initAngle = getHeading(); //getting starting angle

        //waiting until distance traveled >= distance inputted
        while (distance >= getDistanceTraveled(runTime.seconds())) {
            //checking current angle against starting angle and adjusting for any offset greater than 10 deg
            if (getHeading() > initAngle + 10) {
                right(.1, getHeading() - initAngle); //turning right how much we are offset
            }
            else if (getHeading() < initAngle - 10) {
                left(.1, initAngle - getHeading());
            }
        }

        //setting all motor powers to 0 (stopping)
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.frontLeftMotor.setPower(0);

        getAvgVelocity(true); //resetting the average velocity
    }

    //turning right angle (deg) with power [0, 1]
    private void right(double power, double degrees) {

        //setting left motors to go forward (positive power)
        robot.backLeftMotor.setPower(power);
        robot.frontLeftMotor.setPower(power);

        //setting right motors to go backward (negative power)
        robot.backRightMotor.setPower(power); //-
        robot.frontRightMotor.setPower(power); //-

        double initAngle = getHeading(); //angle that the robot is at when it starts

        //wait until angle turned is >= angle inputted
        while (initAngle - getHeading() <= degrees) {
            //do nothing
        }

        //setting motor value to 0 (stop)
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.frontLeftMotor.setPower(0);

    }

    //turning left angle (deg) with power [0, 1]
    private void left(double power, double degrees) {

        //setting left motors to go backward (-power)
        robot.backLeftMotor.setPower(-power);
        robot.frontLeftMotor.setPower(-power);

        //setting right motors to go forward (power)
        robot.backRightMotor.setPower(-power);//+
        robot.frontRightMotor.setPower(-power);//+

        double initAngle = getHeading(); //staring angle

        //wait until angle moved is >= than angle inputted
        while (getHeading() - initAngle <= degrees){
            //do nothing
        }

        //setting motors to 0 (stopping)
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.frontLeftMotor.setPower(0);

    }

    //strafing left distance (m) with power [0, 1]
    private void strafeLeft(double power, long distance) {

        //setting motors in slanted left positive
        robot.backLeftMotor.setPower(power);
        robot.frontRightMotor.setPower(-power);//+

        //setting motors in slanted right negative
        robot.backRightMotor.setPower(power);//-
        robot.frontLeftMotor.setPower(-power);

        runTime.reset(); //staring stopwatch

        double initAngle = getHeading(); //getting starting angle

        //waiting until distance traveled >= distance inputted
        while (distance >= getDistanceTraveled(runTime.seconds())) {
            //checking current angle against starting angle and adjusting for any offset greater than 10 deg
            if (getHeading() > initAngle + 10) {
                right(.1, getHeading() - initAngle); //turning right how much we are offset
            }
            else if (getHeading() < initAngle - 10) {
                left(.1, initAngle - getHeading());
            }
        }

        //setting motor powers to 0 (stopping)
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.frontLeftMotor.setPower(0);

        getAvgVelocity(true); //resetting the average velocity
    }

    //strafing right distance (m) with power [0, 1]
    private void strafeRight(double power, long distance) {

        //setting motors slanted right positive
        robot.backRightMotor.setPower(-power);//+
        robot.frontLeftMotor.setPower(power);

        //setting motors in slanted left negative
        robot.backLeftMotor.setPower(-power);
        robot.frontRightMotor.setPower(power);//-

        runTime.reset(); //staring stopwatch

        double initAngle = getHeading(); //getting starting angle

        //waiting until distance traveled >= distance inputted
        while (distance >= getDistanceTraveled(runTime.seconds())) {
            //checking current angle against starting angle and adjusting for any offset greater than 10 deg
            if (getHeading() > initAngle + 10) {
                right(.1, getHeading() - initAngle); //turning right how much we are offset
            }
            else if (getHeading() < initAngle - 10) {
                left(.1, initAngle - getHeading());
            }
        }

        //setting motor values to 0 (stopping)
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.frontLeftMotor.setPower(0);

        getAvgVelocity(true); //resetting the average velocity
    }

    //moving at an angle (deg) for distance (m) with power [0, 1]
    private void direction(double power, long distance, double degrees) {
        //rotating degrees right by 45 in order to grt angle relative to back left wheel
        double degreesCalc = degrees - 45;

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

        runTime.reset(); //staring stopwatch

        double initAngle = getHeading(); //getting starting angle

        //waiting until distance travelled is >= distance inputted
        while (distance >= getDistanceTraveled(runTime.seconds())) {
            //checking current angle against starting angle and adjusting for any offset greater than 10 deg
            if (getHeading() > initAngle + 10) {
                right(.1, getHeading() - initAngle); //turning right how much we are offset
            }
            else if (getHeading() < initAngle - 10) {
                left(.1, initAngle - getHeading());
            }
        }

        //setting motor values to 0 (stopping)
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.frontLeftMotor.setPower(0);

        getAvgVelocity(true); //resetting the average velocity
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


        telemetry.addLine("wait for gyro calibration"); //telling user to wait for gyro to be callibrated

        //waiting for gyro to calibrate
        while (!imu.isGyroCalibrated()) {
            //waiting
        }

        telemetry.addData("gyro status :: ", imu.getCalibrationStatus().toString()); //returning gyro status (ready to start)

        waitForStart();

        //after start is pressed

        runTime.reset();
        //forward(.2, 1);
        //backward(.2, 1);
        //strafeRight(.2, 1);
        //strafeLeft(.2, 1);
        //right(.2, 90);
        //left(.2, 90);
        //direction(.2, 1, 90);

        while (true) {
            telemetry.addData("distance traveled :: ", getDistanceTraveled(runTime.seconds()));
            telemetry.update();
        }
    }
}