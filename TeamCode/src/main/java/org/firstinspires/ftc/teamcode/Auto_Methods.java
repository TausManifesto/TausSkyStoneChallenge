//code for the autonomous portion of the Skystone challenge

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    double ObjectAngle;
    double TargetHeightRatio;
    int ImageHeight;
    float ObjectHeight;
    double ObjectHeightRatio;

    //Vuforia Key Fragment
    private static final String VUFORIA_KEY = "AY7Mgrn/////AAABmfI+bBY2KUjrj+QipjjQqMZkBy9G2EFK3wmdSctymzd4yHsn91iwwOFsV+MCDHbeaRmEFzBv3MqjDt3prE4YDrozJIUKha18nK9zZVHLMJvE7EE2nyQ2W/DTw045lQDRjRuNjuALhnd9RePTFlNJar/yqkADNUAPR+WNlT+Yb2R29d87B8Q352a7kMNXzglLi+yHtBg1fJwygyuxp8vhlgMd5OFRyBEn3VAtqFxpEVJz5AfdQ6wgKIMwJhSONcHuNmEO6Rvp+QDzKDGtdQ69o2WpRHb9auU/mwTFCjY18ZowZoTfihgoZOHL/g4a4uFOiHqbEQNUNTeulQmVRUuh8iWy2OBwf/Ec/uCEM+RiKzxz";
    ;
    // Initialzing both vuforia and tfod
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    public boolean skystone_detected = false;

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

    private double sigmoid(double error) {

        if (error > 40) {
            return .2;
        } else {
            return .11;
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

        telemetry.addData("degrees turned :: ", initAngle - getHeading());
        telemetry.update();

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
            robot.backRightMotor.setPower(sigmoid(degrees - (initAngle - getHeading()))); //-
            robot.frontRightMotor.setPower(sigmoid(degrees - (initAngle - getHeading()))); //-
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
        String pos = "";
        if (tfod != null) {
            tfod.activate();
        }

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
                                        pos = "Right";
                                        telemetry.addData("Direction", "Right");
                                    }
                                    else if (ObjectAngle < 9 && ObjectAngle > 3) {
                                        pos = "Center";
                                        telemetry.addData("Direction", "Center");
                                    }
                                    else
                                    {
                                        pos = "Left";
                                        telemetry.addData("Direction", "Left");
                                    }
                                    // Using params to detect how close the block is
                                    ImageHeight = recognition.getImageHeight();
                                    ObjectHeight = recognition.getHeight();
                                    // Finding Object Height Ratio
                                    // Large Ratio means that the Robot is Closer
                                    ObjectHeightRatio = ObjectHeight / ImageHeight;
                                    telemetry.addData("HeightRatio", ObjectHeightRatio);
                                    if (ObjectHeightRatio < TargetHeightRatio - 0.05) {
                                        telemetry.addData("Distance: ", "Not Close Enough");
                                    } else if (ObjectHeightRatio > TargetHeightRatio + 0.05) {
                                        telemetry.addData("Distance: ", "Too Close");
                                    } else {
                                        telemetry.addData("Distance: ", "Just Right");
                                    }
                                    break;
                                }
                            }
                            telemetry.update();
                        }
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }

        return pos;
    }

    //Going to skystone
    public void gotoSkystone(String color) {
        if (getSkystonePos() == "Right") {
            strafeRight(.4, 8);
            forward(.4, 35);
            backward(.4,15);
            if (color == "Red"){
                strafeLeft(.4,16);
            }
            if (color == "Red"){
                left(90);
            }
            if (color == "Blue"){
                right(90);
            }
        }
        if (getSkystonePos() == "Left"){
            strafeLeft(.4, 8);
            forward(.4, 35);
            backward(.4, 15);
            if (color == "Blue"){
                strafeRight(.4, 16);
            }
            if (color == "Red"){
                left(90);
            }
            if (color == "Blue"){
                right(90);
            }
        }
        if (getSkystonePos() == "Center"){
            forward(.4, 35);
            backward(.4, 15);
            if (color == "Red"){
                strafeLeft(.4, 8);
            }
            if (color == "Blue") {
                strafeRight(.4, 8);
            }
            if (color == "Red"){
                left(90);
            }
            if (color == "Blue"){
                right(90);
            }
        }
    }

    public void runOpMode() throws InterruptedException {}
}