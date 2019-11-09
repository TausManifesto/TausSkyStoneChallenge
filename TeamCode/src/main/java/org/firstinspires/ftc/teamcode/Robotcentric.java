package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name = "Tau TeleOp", group = "Tau")
public class Robotcentric extends OpMode {
    Hardware robot = new Hardware();
    ElapsedTime period = new ElapsedTime();

    private double frontleftPOWER = 0;
    private double frontrightPOWER = 0;
    private double backleftPOWER = 0;
    private double backrightPOWER = 0;


    private double liftPower = 0;
    private double extensionPower = 0;




    private double leftIntakePower = 0;
    private double rightIntakePower = 0;

    private double maxPOWER = 1;
    private final double triggerConstant = 0.75;

    private double leftGP1Y = 0;
    private double leftGP1X = 0;
    private double rightGP1X = 0;

    private double leftGP2X = 0;
    private double rightGP2Y = 0;


    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);
        telemetry.addData("Readiness", "NOT READY TO START, PLEASE WAIT");
        telemetry.update();
        telemetry.addData("Readiness", "Press Play to start");
        telemetry.addData("If you notice this", "You are COOL!!! (Charles was here)");
        telemetry.update();
    }
    @Override
    public void init_loop()
    {

    }

    @Override
    public void start()
    {

    }

    @Override
    public void loop() {


        leftGP1X = gamepad1.left_stick_x;
        leftGP1Y = -gamepad1.left_stick_y;
        rightGP1X = gamepad1.right_stick_x;





        if (Math.abs(leftGP1Y) < 0.05) {
            leftGP1Y = 0;
        }
        if (Math.abs(leftGP1X) < 0.05) {
            leftGP1X = 0;
        }

        if (Math.abs(rightGP1X) < 0.05) {
            rightGP1X = 0;
        }




        if (Math.abs(leftGP2X) < 0.05) {
            leftGP2X = 0;
        }
        if (Math.abs(rightGP2Y) < 0.05) {
            rightGP2Y = 0;
        }

        backleftPOWER = leftGP1Y - leftGP1X + rightGP1X;
        backrightPOWER = -leftGP1Y - leftGP1X + rightGP1X;
        frontleftPOWER = leftGP1Y + leftGP1X + rightGP1X;
        frontrightPOWER = -leftGP1Y + leftGP1X + rightGP1X;


//        if (gamepad1.left_trigger > 0.05) {
//            frontrightPOWER = frontrightPOWER - gamepad1.left_trigger * triggerConstant;
//            frontleftPOWER = frontleftPOWER - gamepad1.left_trigger * triggerConstant;
//            backrightPOWER = backrightPOWER - gamepad1.left_trigger * triggerConstant;
//            backleftPOWER = backleftPOWER - gamepad1.left_trigger * triggerConstant;
//        }
//        if (gamepad1.right_trigger > 0.05) {
//            frontrightPOWER = frontrightPOWER + gamepad1.right_trigger * triggerConstant;
//            frontleftPOWER = frontleftPOWER + gamepad1.right_trigger * triggerConstant;
//            backrightPOWER = backrightPOWER + gamepad1.right_trigger * triggerConstant;
//            backleftPOWER = backleftPOWER + gamepad1.right_trigger * triggerConstant;
//        }


        maxPOWER = Math.abs(frontleftPOWER);

        if (Math.abs(backleftPOWER) > maxPOWER) {
            maxPOWER = Math.abs(backleftPOWER);
        }
        if (Math.abs(backrightPOWER) > maxPOWER) {
            maxPOWER = Math.abs(backrightPOWER);
        }
        if (Math.abs(frontrightPOWER) > maxPOWER) {
            maxPOWER = Math.abs(frontrightPOWER);
        }

        if (maxPOWER > 1.0) {
            frontrightPOWER = frontrightPOWER / maxPOWER;
            frontleftPOWER = frontleftPOWER / maxPOWER;
            backrightPOWER = backrightPOWER / maxPOWER;
            backleftPOWER = backleftPOWER / maxPOWER;
        }


        robot.frontLeftMotor.setPower(frontleftPOWER);
        robot.frontRightMotor.setPower(frontrightPOWER);
        robot.backLeftMotor.setPower(backleftPOWER);
        robot.backRightMotor.setPower(backrightPOWER);

        if(gamepad2.x ){
            robot.clawServo.setPower(0.7);
            telemetry.addData("grabbing block", gamepad2.x);
        } else if(gamepad2.a){
            telemetry.addData("releasing block", gamepad2.x);
            robot.clawServo.setPower((-0.2));
        }else {
            robot.clawServo.setPower(0.0);
        }



        if (Math.abs(gamepad2.right_trigger) > 0.05) {
            telemetry.addData("extension", gamepad2.right_trigger);
            extensionPower = gamepad2.right_trigger > 0 ? 0.4 : -0.4;
        } else{
            extensionPower = 0.0;
        }
        robot.leftExtensionServo.setPower(extensionPower);
        robot.rightExtensionServo.setPower(extensionPower);




        if (Math.abs(gamepad2.left_trigger) > 0.07 && !robot.leftLiftLimit.isPressed()) {
            telemetry.addData("lift", gamepad2.left_trigger);
            liftPower = gamepad2.left_trigger > 0 ? 0.5 : -0.5;
        } else {
            liftPower = 0.0;
        }
        robot.leftLiftServo.setPower(liftPower);
        robot.rightLiftServo.setPower(liftPower);




    }






}
