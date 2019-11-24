//code for the autonomous portion of the Skystone challenge

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * created by ashwin jandhyala
 * 10/6/2019
 */
//@Disabled

public class Auto_Methods extends LinearOpMode {

    // Declaring Labels and ObjDetect Models
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    //Vuforia Key Fragment

    private static final String VUFORIA_KEY = "AY7Mgrn/////AAABmfI+bBY2KUjrj+QipjjQqMZkBy9G2EFK3wmdSctymzd4yHsn91iwwOFsV+MCDHbeaRmEFzBv3MqjDt3prE4YDrozJIUKha18nK9zZVHLMJvE7EE2nyQ2W/DTw045lQDRjRuNjuALhnd9RePTFlNJar/yqkADNUAPR+WNlT+Yb2R29d87B8Q352a7kMNXzglLi+yHtBg1fJwygyuxp8vhlgMd5OFRyBEn3VAtqFxpEVJz5AfdQ6wgKIMwJhSONcHuNmEO6Rvp+QDzKDGtdQ69o2WpRHb9auU/mwTFCjY18ZowZoTfihgoZOHL/g4a4uFOiHqbEQNUNTeulQmVRUuh8iWy2OBwf/Ec/uCEM+RiKzxz";
    public boolean skystone_detected = false;
    double ObjectAngle;
    double LeftSide;
    double TargetHeightRatio;
    int ImageHeight;
    float ObjectHeight;
    double ObjectHeightRatio;
    // Initialzing both vuforia and tfod
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private BNO055IMU imu;

    private Hardware robot = new Hardware();

    public void initRobot() {
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
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        // Activating ...
        if (tfod != null) {
            tfod.activate();
        }

        waitForStart();
    }

    //defining methods for returning imu reading

    //getting angle based on initial angle
    private double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle; //right :: [0, -180] -- left :: [0, 180]
    }

    private double sigmoid(double error) {
        //return 1.4 / (1 + Math.pow(Math.E, -error)/20) - .7;
        return .15;
    }


    //defining Methods for the robot

    //moving forward distance (m) with power [0, 1]
    public void forward(double power, double distance) {
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int counts = (int) ((distance / (4 * Math.PI)) * 1075);
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

        while (opModeIsActive() && robot.backLeftMotor.isBusy() && robot.backRightMotor.isBusy() && robot.frontRightMotor.isBusy() && robot.frontLeftMotor.isBusy()) {
            idle();
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
    public void backward(double power, double distance) {
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int counts = (int) ((distance / (4 * Math.PI)) * 1075);
        robot.backLeftMotor.setTargetPosition(-counts);
        robot.backRightMotor.setTargetPosition(-counts);
        robot.frontRightMotor.setTargetPosition(-counts);
        robot.frontLeftMotor.setTargetPosition(-counts);

        //setting all motors to go forward (positive)

        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftMotor.setPower(-power);
        robot.backRightMotor.setPower(-power);
        robot.frontRightMotor.setPower(-power);
        robot.frontLeftMotor.setPower(-power);

        while (opModeIsActive() && robot.backLeftMotor.isBusy() && robot.backRightMotor.isBusy() && robot.frontRightMotor.isBusy() && robot.frontLeftMotor.isBusy()) {
            idle();
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
    public void right(double degrees) {

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double initAngle = getHeading(); //angle that the robot is at when it starts

        //wait until angle turned is >= angle inputted
        while (initAngle - getHeading() <= degrees) {

            //setting left motors to go forward (positive power)
            robot.backLeftMotor.setPower(sigmoid(degrees - (initAngle - getHeading())));
            robot.frontLeftMotor.setPower(sigmoid(degrees - (initAngle - getHeading())));

            //setting right motors to go backward (negative power)
            robot.backRightMotor.setPower(-sigmoid(degrees - (initAngle - getHeading()))); //-
            robot.frontRightMotor.setPower(-sigmoid(degrees - (initAngle - getHeading()))); //-
        }

        //setting motor value to 0 (stop)
        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    //turning left angle (deg) with power [0, 1]
    public void left(double degrees) {

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double initAngle = getHeading(); //angle that the robot is at when it starts

        //wait until angle turned is >= angle inputted
        while (getHeading() - initAngle <= degrees) {

            //setting left motors to go forward (positive power)
            robot.backLeftMotor.setPower(-sigmoid(degrees - (getHeading() - initAngle)));
            robot.frontLeftMotor.setPower(-sigmoid(degrees - (getHeading() - initAngle)));

            //setting right motors to go backward (negative power)
            robot.backRightMotor.setPower(sigmoid(degrees - (initAngle - getHeading())));
            robot.frontRightMotor.setPower(sigmoid(degrees - (initAngle - getHeading())));
        }

        //setting motor value to 0 (stop)
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    //strafing left distance (m) with power [0, 1]
    public void strafeLeft(double power, long distance) {
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int counts = (int) ((distance / (4 * Math.PI)) * 1075);
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

        while (opModeIsActive() && robot.backLeftMotor.isBusy() && robot.backRightMotor.isBusy() && robot.frontRightMotor.isBusy() && robot.frontLeftMotor.isBusy()) {
            idle();
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
    public void strafeRight(double power, long distance) {
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int counts = (int) ((distance / (4 * Math.PI)) * 1075);
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

        while (opModeIsActive() && robot.backLeftMotor.isBusy() && robot.backRightMotor.isBusy() && robot.frontRightMotor.isBusy() && robot.frontLeftMotor.isBusy()) {
            idle();
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

    public void liftUp(double power, int time) {
        robot.leftLiftServo.setPower(power);
        robot.rightLiftServo.setPower(power);
        sleep(time);
        robot.rightLiftServo.setPower(.1);
        robot.leftLiftServo.setPower(.1);
    }

    public void liftDrop() {
        robot.rightLiftServo.setPower(0);
        robot.leftLiftServo.setPower(0);
    }

    public void liftOut(double power, int time) {
        robot.leftExtensionServo.setPower(power);
        robot.rightExtensionServo.setPower(power);
        sleep(time);
        robot.rightExtensionServo.setPower(0);
        robot.leftExtensionServo.setPower(0);
    }

    public void liftDown(double power, int time) {
        robot.leftLiftServo.setPower(-power);
        robot.rightLiftServo.setPower(-power);
        sleep(time);
        robot.rightLiftServo.setPower(0);
        robot.leftLiftServo.setPower(0);
    }

    //Initializing Vuforia
    public void initVuforia() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    //Loading Models ....
    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    //Finding and returning position of Skystone
    public String getSkystonePosRed() {

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> recognitions = tfod.getUpdatedRecognitions();
                    //telemetry.addData("# Object Detected", recognitions.size());
                    int x = 0;
                    if (recognitions != null) {
                        if (recognitions.size() > 0) {
                            telemetry.addData("# Object Detected", recognitions.size());
                            // step through the list of recognitions and display boundary info.
                            for (Recognition recognition : recognitions) {
                                x++;
                                if (recognition.getLabel().equals("Skystone")) {
                                    telemetry.addData("count ", x);
                                    // finding if skystone is there or not
                                    telemetry.addData("Skystone: ", "Detected");
                                    skystone_detected = true;
                                    // getting provided estimated angle
                                    ObjectAngle = recognition.estimateAngleToObject(AngleUnit.DEGREES);
                                    LeftSide = recognition.getLeft();
                                    //RightSide = recognitions.get(i).getRight();
                                    // showing user object angle
                                    telemetry.addData("Estimated Angle", ObjectAngle);
                                    telemetry.addData("Position of Left side", LeftSide);

                                    if (ObjectAngle >= 0) {
                                        telemetry.addData("Direction", "Right");
                                        return "Right";
                                    } else if (ObjectAngle <= 0) {
                                        telemetry.addData("Direction", "Center");
                                        return "Center";
                                    } else {
                                        telemetry.addData("Direction", "Left");
                                        return "left";
                                    }

                                }


                            }
                            telemetry.update();
                        }
                    }
                }
            }

        }

        return "ERROR";
    }

    //Finding degrees of Skystone
    public String getSkystonePosBlue() {

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> recognitions = tfod.getUpdatedRecognitions();
                    if (recognitions != null) {
                        if (recognitions.size() > 0) {
                            telemetry.addData("# Object Detected", recognitions.size());
                            // step through the list of recognitions and display boundary info.
                            for (int i = 0; i < recognitions.size(); i++) {
                                if (recognitions.get(i).getLabel().equals("Skystone")) {
                                    // finding if skystone is there or not
                                    telemetry.addData("Skystone: ", "Detected");
                                    skystone_detected = true;
                                    // getting provided estimated angle
                                    ObjectAngle = recognitions.get(i).estimateAngleToObject(AngleUnit.DEGREES);
                                    // showing user object angle
                                    telemetry.addData("Estimated Angle", ObjectAngle);

                                    if (ObjectAngle >= 0) {
                                        telemetry.addData("Direction", "Center");
                                        return "Center";
                                    } else if (ObjectAngle <= 0) {
                                        telemetry.addData("Direction", "Left");
                                        return "Left";
                                    } else {
                                        return "Right";
                                    }
                                }
                            }
                            telemetry.update();
                        }
                    }
                }
            }

        }

        return "ERROR";
    }
    //Going to skystone

    public void gotoSkystone(String color, String pos) {

        if (pos.equals("Left")) {
            if (color.equals("Red")) {
                strafeLeft(.5, 16);
            }
            forward(.5, 35);
            backward(.5, 15);
            if (color.equals("Red")) {
                left(90);
                backward(.5, 16);
            }
            if (color.equals("Blue")) {
                right(90);
            }
        } else if (pos.equals("Right")) {
            if (color.equals("Blue")) {
                strafeRight(.5, 16);
            }
            forward(.5, 35);
            backward(.5, 15);
            if (color.equals("Red")) {
                left(90);
            }
            if (color.equals("Blue")) {
                right(90);
                backward(.5, 16);
            }
        } else if (pos.equals("Center")) {
            if (color.equals("Red")) {
                strafeLeft(.5, 8);
            } else {
                strafeRight(.5, 8);
            }
            forward(.5, 35);
            backward(.5, 15);
            if (color.equals("Red")) {
                left(90);
                backward(.5, 8);
            }
            if (color.equals("Blue")) {
                right(90);
                backward(.5, 8);
            }
        }
    }

    public void gotoSkystone2(String color, String pos) {

        if (pos.equals("Left")) {
            if (color.equals("Red")) {
                strafeLeft(.5, 16);
            }
            forward(.5, 15);
            left(180);
            forward(.5, 25);
            if (color.equals("Red")) {
                left(90);
                forward(.5, 16);
            }
            if (color.equals("Blue")) {
                right(90);
            }
        } else if (pos.equals("Right")) {
            if (color.equals("Blue")) {
                strafeRight(.5, 16);
            }
            forward(.5, 15);
            left(180);
            forward(.5, 25);
            if (color.equals("Red")) {
                left(90);
            }
            if (color.equals("Blue")) {
                right(90);
                forward(.5, 16);
            }
        } else if (pos.equals("Center")) {
            if (color.equals("Red")) {
                strafeLeft(.5, 8);
            } else {
                strafeRight(.5, 8);
            }
            forward(.5, 15);
            left(180);
            backward(.5, 25);
            if (color.equals("Red")) {
                left(90);
                forward(.5, 8);
            }
            if (color.equals("Blue")) {
                right(90);
                forward(.5, 8);
            }
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
    }
}

