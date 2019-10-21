package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2019-2020 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Skystone game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Disabled
@Autonomous(name = "ConceptTensorFlowObjectDetection", group = "Concept")

public class ConceptTensorFlowObjectDetection extends LinearOpMode {
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


    @Override
    public void runOpMode() {

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

        /** Wait for the game to begin */
        waitForStart();

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
                                        telemetry.addData("Direction", "Right");
                                    }
                                    else if (ObjectAngle < 9 && ObjectAngle > 3) {
                                        telemetry.addData("Direction", "Center");
                                    }
                                    else
                                     {
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
    }

    // Initializing Vuforia
    private void initVuforia() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    // Loading Models ....
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.6;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}







//
//    /*
//     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
//     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
//     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
//     * web site at https://developer.vuforia.com/license-manager.
//     *
//     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
//     * random data. As an example, here is a example of a fragment of a valid key:
//     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
//     * Once you've obtained a license key, copy the string from the Vuforia web site
//     * and paste it in to your code on the next line, between the double quotes.
//     */
//    private static final String VUFORIA_KEY =
//            "AY7Mgrn/////AAABmfI+bBY2KUjrj+QipjjQqMZkBy9G2EFK3wmdSctymzd4yHsn91iwwOFsV+MCDHbeaRmEFzBv3MqjDt3prE4YDrozJIUKha18nK9zZVHLMJvE7EE2nyQ2W/DTw045lQDRjRuNjuALhnd9RePTFlNJar/yqkADNUAPR+WNlT+Yb2R29d87B8Q352a7kMNXzglLi+yHtBg1fJwygyuxp8vhlgMd5OFRyBEn3VAtqFxpEVJz5AfdQ6wgKIMwJhSONcHuNmEO6Rvp+QDzKDGtdQ69o2WpRHb9auU/mwTFCjY18ZowZoTfihgoZOHL/g4a4uFOiHqbEQNUNTeulQmVRUuh8iWy2OBwf/Ec/uCEM+RiKzxz";
//
//    /**
//     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
//     * localization engine.
//     */
//    private VuforiaLocalizer vuforia;
//
//    /**
//     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
//     * Detection engine.
//     */
//    private TFObjectDetector tfod;
//
//    @Override
//    public void runOpMode() {
//        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
//        // first.
//        initVuforia();
//
//        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
//            initTfod();
//        } else {
//            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
//        }
//
//        /**
//         * Activate TensorFlow Object Detection before we wait for the start command.
//         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
//         **/
//        if (tfod != null) {
//            tfod.activate();
//        }
//
//        /** Wait for the game to begin */
//        telemetry.addData(">", "Press Play to start op mode");
//        telemetry.update();
//        waitForStart();
//
//        /* if (opModeIsActive()) {
//            while (opModeIsActive()) {
//                if (tfod != null) {
//                    // getUpdatedRecognitions() will return null if no new information is available since
//                    // the last time that call was made.
//                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//                    if (updatedRecognitions != null) {
//                        telemetry.addData("# Object Detected", updatedRecognitions.size());
//
//                        // step through the list of recognitions and display boundary info.
//                        int i = 0;
//                        for (Recognition recognition : updatedRecognitions) {
//                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
//                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
//                                    recognition.getLeft(), recognition.getTop());
//                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
//                                    recognition.getRight(), recognition.getBottom());
//                        }
//                        telemetry.update();
//                    }
//                }
//            }
//        } */
//
//
//        while (opModeIsActive()) {
//            if (tfod != null) {
//                // getUpdatedRecognitions() will return null if no new information is available since
//                // the last time that call was made.
//                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//                if (updatedRecognitions != null) {
//                    telemetry.addData("# Object Detected", updatedRecognitions.size());
//
//                    //if you scan more than three blocks, then take the first three blocks
//
////                    if(updatedRecognitions.size() > 3)
////                    {
////                        updatedRecognitions = updatedRecognitions.subList(0,3);
////
////                    }
//
//                    if(updatedRecognitions.size() == 3) {
//                    int goldMineralX = -1;
//                    int silverMineral1X = -1;
//                    int silverMineral2X = -1;
//                    for (Recognition recognition : updatedRecognitions) {
//                        float center = (recognition.getLeft() + recognition.getRight())/2;
//                        if (recognition.getLabel().equals(LABEL_SKYSTONE)) {
//                            goldMineralX = (int) center;
//                        } else if (silverMineral1X == -1) {
//                            silverMineral1X = (int) center;
//                        } else {
//                            silverMineral2X = (int) center;
//                        }
//                        //telemetry.addData("Angle to Object", recognition.estimateAngleToObject(AngleUnit.DEGREES));
//                    }
//                    if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
//                        if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
//                            telemetry.addData("SkyStone Position", "Left");
//                        } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
//                            telemetry.addData("SkyStone Position", "Right");
//                        } else {
//                            telemetry.addData("SkyStone Position", "Center");
//                        }
//                    }
//                }
//                    telemetry.update();
//                }
//            }
//        }
//​
//        if (tfod != null) {
//            tfod.shutdown();
//        }
//    }​
    // Initializing Vuforia
//    private void initVuforia() {
//​
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//​
//        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//        parameters.cameraName = hardwareMap.get(WebcamName.class, "head_webcam");
//​
//        //  Instantiate the Vuforia engine
//        vuforia = ClassFactory.getInstance().createVuforia(parameters);
//​
//        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
//    }
//​
//    // Loading Models ....
//    private void initTfod() {
//        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//        tfodParameters.minimumConfidence = 0.7;
//        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
//        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_STONE, LABEL_SKYSTONE);




