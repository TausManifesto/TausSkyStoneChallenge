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

        if (error > 60) {
            return .2;
        } else if (error > 40) {
            return .1;
        } else {
            return .075;
        }
        //return  1/(1 + Math.pow(3, -error/9))-.5;
    }


    //defining Methods for the robot

    //moving forward distance (m) with power [0, 1]
    public void forward(double power, double distance) {

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
    public void backward(double power, long distance) {

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
            robot.backLeftMotor.setPower(-sigmoid(degrees - (initAngle - getHeading())));
            robot.frontLeftMotor.setPower(-sigmoid(degrees - (initAngle - getHeading())));

            //setting right motors to go backward (negative power)
            robot.backRightMotor.setPower(sigmoid(degrees - (initAngle - getHeading())));
            robot.frontRightMotor.setPower(sigmoid(degrees - (initAngle - getHeading())));
        }

        //setting motor value to 0 (stop)
        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

    //strafing left distance (m) with power [0, 1]
    public void strafeLeft(double power, long distance) {
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

    //Initializing Vuforia
    public void initVuforia() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    //Loading Models ....
    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.6;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    //Finding and returning position of Skystone
    public String getSkystonePos() {

        boolean yes = true;
        String pos = "error";

        if (opModeIsActive()) {
            while (opModeIsActive() && yes) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> recognitions = tfod.getUpdatedRecognitions();
                    if (recognitions != null) {
                        if (recognitions.size() > 0) {
                            telemetry.addData("# Object Detected", recognitions.size());
                            telemetry.update();
                            sleep(5000);
                            // step through the list of recognitions and display boundary info.
                            for (int i = 0; i < recognitions.size(); i++) {
                                if (recognitions.get(i).getLabel() == "Skystone") {
                                    // finding if skystone is there or not
                                    telemetry.addData("Skystone: ", "Detected");
                                    telemetry.update();
                                    skystone_detected = true;
                                    // getting provided estimated angle
                                    ObjectAngle = recognitions.get(i).estimateAngleToObject(AngleUnit.DEGREES);
                                    // showing user object angle
                                    telemetry.addData("Estimated Angle", ObjectAngle);
                                    telemetry.update();
                                    if (ObjectAngle > 7) {
                                        telemetry.addData("Direction", "Right");
                                        pos = "Right";
                                    } else if (ObjectAngle < 7 && ObjectAngle > -1) {
                                        telemetry.addData("Direction", "Center");
                                        pos = "Center";
                                    } else {
                                        telemetry.addData("Direction", "Left");
                                        pos = "Left";
                                    }
                                    yes = false;
                                    break;
                                }
                            }
                            telemetry.update();
                        }
                    }
                }
            }

        }

        return pos;
    }

    //Finding degrees of Skystone
    public Double getSkystoneAngle() {

        boolean yes = true;
        double deg = 100;

        if (opModeIsActive()) {
            while (opModeIsActive() && yes) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> recognitions = tfod.getUpdatedRecognitions();
                    if (recognitions != null) {
                        if (recognitions.size() > 0) {
                            telemetry.addData("# Object Detected", recognitions.size());
                            // step through the list of recognitions and display boundary info.
                            for (Recognition recognition : recognitions) {
                                if (recognition.getLabel() == "Skystone") {
                                    // finding if skystone is there or not
                                    telemetry.addData("Skystone: ", "Detected");
                                    skystone_detected = true;
                                    // getting provided estimated angle
                                    ObjectAngle = recognition.estimateAngleToObject(AngleUnit.DEGREES);
                                    // showing user object angle
                                    telemetry.addData("Estimated Angle", ObjectAngle);
                                    if (ObjectAngle > 9) {
                                        telemetry.addData("Direction", "Right");
                                    } else if (ObjectAngle < 9 && ObjectAngle > 3) {
                                        telemetry.addData("Direction", "Center");
                                    } else {
                                        telemetry.addData("Direction", "Left");
                                    }
                                    deg = ObjectAngle;
                                    yes = false;
                                    break;
                                }
                            }
                            telemetry.update();
                        }
                    }
                }
            }

        }

        return deg;
    }

    //Going to skystone
    public void gotoSkystone(String color, String pos) {

        if (pos.equals("Left")) {
            if (color.equals("Blue")) {
                strafeLeft(.2, 5);
            } else {
                strafeLeft(.2, 15);
            }
            forward(.2, 35);
            backward(.2, 15);
            if (color.equals("Red")) {
                left(90);
                backward(.2, 19);
            }
            if (color.equals("Blue")) {
                right(90);
            }
        } else if (pos.equals("Right")) {
            if (color.equals("Blue")) {
                strafeRight(.2, 15);
            } else {
                strafeRight(.2, 5);
            }
            forward(.2, 35);
            backward(.2, 15);
            if (color.equals("Red")) {
                left(90);
            }
            if (color.equals("Blue")) {
                right(90);
                backward(.2, 19);
            }
        } else if (pos.equals("Center")) {
            forward(.2, 35);
            backward(.2, 15);
            if (color.equals("Red")) {
                strafeLeft(.2, 5);
            }
            if (color.equals("Blue")) {
                strafeRight(.2, 16);
            }
            if (color.equals("Red")) {
                left(90);
            }
            if (color.equals("Blue")) {
                right(90);
            }
            if (color.equals("Red")) {
                backward(.2, 8);
            }
            if (color.equals("Blue")) {
                backward(.2, 16);
            }
        }
    }

    //Going to skystone
    public void gotoSkystone2(String color, String pos) {

        if (pos.equals("Left")) {
            if (color.equals("Blue")) {
                strafeLeft(.2, 5);
            } else {
                left(10);
            }
            forward(.2, 35);
            backward(.2, 15);
            if (color.equals("Red")) {
                left(80);
                backward(.2, 19);
            }
            if (color.equals("Blue")) {
                right(90);
            }
        } else if (pos.equals("Right")) {
            if (color.equals("Blue")) {
                right(10);
            } else {
                strafeRight(.2, 5);
            }
            forward(.2, 35);
            backward(.2, 15);
            if (color.equals("Red")) {
                left(90);
            }
            if (color.equals("Blue")) {
                right(80);
                backward(.2, 19);
            }
        } else if (pos.equals("Center")) {
            forward(.2, 35);
            backward(.2, 15);
            if (color.equals("Red")) {
                strafeLeft(.2, 5);
            }
            if (color.equals("Blue")) {
                strafeRight(.2, 16);
            }
            if (color.equals("Red")) {
                left(90);
            }
            if (color.equals("Blue")) {
                right(90);
            }
            if (color.equals("Red")) {
                backward(.2, 8);
            }
            if (color.equals("Blue")) {
                backward(.2, 16);
            }
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
    }
}