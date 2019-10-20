//code for the autonomous portion of the Skystone challenge

package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;


/**
 * created by ashwin jandhyala
 * 10/6/2019
 */
//@Disabled
@Autonomous(name = "2019_FTC_Auto_Enc_Vision", group = "Tau")

public class Auto_Mechanum_Enc_Vision extends Auto_Methods {

    private BNO055IMU imu;
    //@Override
    private ElapsedTime runTime = new ElapsedTime();

    private Hardware robot = new Hardware();


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

        initVuforia();
        TargetHeightRatio = 0.8;

        // Creating Tfod Instance
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        }
        else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        waitForStart();

        //after start is pressed

        //runTime.reset();

        //red -- foundation
        //forward(.4, 30);
        //sleep(1000);
        //backward(.4, 22);
        //sleep(1000);
        strafeLeft(.4, 50);
        //sleep(1000);

        //red -- blocks
        //gotoSkystone("Red");
        //forward(.4,50);

        //blue -- foundation
        //forward(.4, 30);
        //sleep(1000);
        //backward(.4, 22);
        //sleep(1000);
        //strafeRight(.4, 50);
        //sleep(1000);

        //blue -- blocks
        //gotoSkystone("Blue");
        //forward(.4, 50);


    }
}