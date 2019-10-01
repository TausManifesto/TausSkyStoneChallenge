package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.util.ElapsedTime;


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
    double xVelcoty;
    double yVelcoty;
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


    Hardware robot = new Hardware();


    public double getHeading()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = angles.firstAngle;
        return heading;
    }

    public double getVelocityX()
    {
        Velocity velocity = imu.getVelocity();
        xVelcoty = velocity.xVeloc;
        return xVelcoty;
    }

    public double getVelocityY()
    {
        Velocity velocity = imu.getVelocity();
        yVelcoty = velocity.yVeloc;
        return yVelcoty;

    }

    public double getVelocity()
    {
        velocity = Math.sqrt(Math.pow(getVelocityX(), 2) + Math.pow(getVelocityY(), 2));
        return velocity;
    }

    public double getDistanceTraveled(double time)
    {
        distTraveled = getVelocity()*time;
        return distTraveled;
    }

    public double getDistanceTraveledX(double time)
    {
        distTraveledX = getVelocityX()*time;
        return distTraveledX;
    }

    public double getDistanceTraveledY(double time)
    {
        distTraveledY = getVelocityY()*time;
        return distTraveledY;
    }

    public double getdirectionTraveled()
    {
        direcTraveled = Math.atan(getVelocityY()/getVelocityX());
        return direcTraveled;
    }


    //Defining Methods for the robot

    public void forward(double power,double distance)
    {
        robot.backLeftMotor.setPower(power);

        robot.backRightMotor.setPower(-power);//+

        robot.frontRightMotor.setPower(-power);//+

        robot.frontLeftMotor.setPower(power);

        runTime.reset();

        initAngle = getHeading();

        while(distance<getDistanceTraveled(runTime.seconds())){
            if(getHeading()>initAngle+5){
                left(.1, getHeading()-initAngle);
            }
            else if(getHeading()<initAngle-5){
                left(.1, initAngle-getHeading());
            }
            distX = getDistanceTraveledX(time);
            distY = getDistanceTraveledY(time);
        }



        robot.backLeftMotor.setPower(0);

        robot.backRightMotor.setPower(0);

        robot.frontRightMotor.setPower(0);

        robot.frontLeftMotor.setPower(0);

        currentX += distX;
        currentY += distY;

    }

    public void backward(double power,long distance)
    {
        robot.backLeftMotor.setPower(-power);

        robot.backRightMotor.setPower(power);//-

        robot.frontRightMotor.setPower(power);//-

        robot.frontLeftMotor.setPower(-power);

        runTime.reset();

        double time = (runTime.seconds());

        while(distance<getDistanceTraveled(time)){
            if(getHeading()>5){
                right(.1, getHeading());
            }
            else if(getHeading()<5){
                left(.1, getHeading());
            }
            distX = getDistanceTraveledX(time);
            distY = getDistanceTraveledY(time);
        }

        robot.backLeftMotor.setPower(0);

        robot.backRightMotor.setPower(0);

        robot.frontRightMotor.setPower(0);

        robot.frontLeftMotor.setPower(0);

        currentX += distX;
        currentY += distY;

    }

    public void right(double power,double degrees)
    {

        robot.backLeftMotor.setPower(power);

        robot.backRightMotor.setPower(power);//-

        robot.frontRightMotor.setPower(power);//-

        robot.frontLeftMotor.setPower(power);

        while (Math.abs(getHeading())<(Math.abs(degrees))){

        }

        robot.backLeftMotor.setPower(0);

        robot.backRightMotor.setPower(0);

        robot.frontRightMotor.setPower(0);

        robot.frontLeftMotor.setPower(0);

    }

    public void left(double power,double degrees)
    {

        robot.backLeftMotor.setPower(-power);

        robot.backRightMotor.setPower(-power);//+

        robot.frontRightMotor.setPower(-power);//+

        robot.frontLeftMotor.setPower(-power);

        while (Math.abs(getHeading())<(Math.abs(degrees))){

        }

        robot.backLeftMotor.setPower(0);

        robot.backRightMotor.setPower(0);

        robot.frontRightMotor.setPower(0);

        robot.frontLeftMotor.setPower(0);

    }

    public void strafeLeft(double power,long distance)
    {
        robot.backLeftMotor.setPower(power);

        robot.backRightMotor.setPower(power);//-

        robot.frontRightMotor.setPower(-power);//+

        robot.frontLeftMotor.setPower(-power);

        runTime.reset();

        while(distance<getDistanceTraveled(runTime.seconds())){
            if(getHeading()>5){
                right(.1, getHeading());
            }
            else if(getHeading()<5){
                left(.1, getHeading());
            }
            distX = getDistanceTraveledX(time);
            distY = getDistanceTraveledY(time);
        }

        robot.backLeftMotor.setPower(0);

        robot.backRightMotor.setPower(0);

        robot.frontRightMotor.setPower(0);

        robot.frontLeftMotor.setPower(0);

        currentX += distX;
        currentY += distY;

    }

    public void strafeRight(double power,long distance)
    {
        robot.backLeftMotor.setPower(-power);

        robot.backRightMotor.setPower(-power);//+

        robot.frontRightMotor.setPower(power);//-

        robot.frontLeftMotor.setPower(power);

        runTime.reset();

        while(distance<getDistanceTraveled(runTime.seconds())){
            if(getHeading()>5){
                right(.1, getHeading());
            }
            else if(getHeading()<5){
                left(.1, getHeading());
            }
            distX = getDistanceTraveledX(time);
            distY = getDistanceTraveledY(time);
        }

        robot.backLeftMotor.setPower(0);

        robot.backRightMotor.setPower(0);

        robot.frontRightMotor.setPower(0);

        robot.frontLeftMotor.setPower(0);

        currentX += distX;
        currentY += distY;

    }

    public void direction(double power,long distance,double degrees)
    {
        degreesCalc=degrees-45;

        double xVector = power*Math.cos(degreesCalc);

        double yVector = power*Math.sin(degreesCalc);

        robot.backLeftMotor.setPower(xVector);

        robot.backRightMotor.setPower(-yVector);//+

        robot.frontRightMotor.setPower(-xVector);//+

        robot.frontLeftMotor.setPower(yVector);

        runTime.reset();

        while(distance<getDistanceTraveled(runTime.seconds())){
            if(getHeading()>5){
                right(.1, getHeading());
            }
            else if(getHeading()<5){
                left(.1, getHeading());
            }
        }

        robot.backLeftMotor.setPower(0);

        robot.backRightMotor.setPower(0);

        robot.frontRightMotor.setPower(0);

        robot.frontLeftMotor.setPower(0);

    }


    @Override
    public void runOpMode()
    {
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu.initialize(parameters);

        robot.initTeleOpNOIMU(hardwareMap);

        waitForStart();

        forward(.5, 10);

        //backward(.1, 10000);

        //strafeRight(.1, 10000);

        //direction(1, 1000, 50);
    }

}
//    public void sleepTau(long milliSec)
//    {
//        try{Thread.sleep(milliSec);}catch(InterruptedException e){throw new RuntimeException(e);}
//    }