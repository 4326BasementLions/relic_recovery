/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.hardware.ColorSensor;
import android.graphics.Color;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import android.app.Activity;
import android.view.View;

//Vuforia Stuff

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Stew's Autonomous", group="Autonomous")

public class aut_encoder extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    static final double     STRAFE_SPEED              = 0.5;

    //Init Sensors
//    ColorSensor whiteLine;
    ColorSensor ballSensor;
//    GyroSensor gyroSensor;

    //Init Motors
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;

    //Init Servos
    Servo armServo;

    Servo rightArm;
    Servo leftArm;

    //Vuforia
    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    //    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
    VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
    VuforiaTrackable relicTemplate = relicTrackables.get(0);
    @Override
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        //init
        leftFront = hardwareMap.dcMotor.get("left front");
        rightFront = hardwareMap.dcMotor.get("right front");
        leftBack = hardwareMap.dcMotor.get("left back");
        rightBack = hardwareMap.dcMotor.get("right back");

//        whiteLine = hardwareMap.colorSensor.get("whiteLine");
        ballSensor = hardwareMap.colorSensor.get("ballSensor");
        armServo = hardwareMap.servo.get("armServo");
        rightArm = hardwareMap.servo.get("rightArm");
        leftArm = hardwareMap.servo.get("leftArm");


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                leftFront.getCurrentPosition(),
                rightFront.getCurrentPosition(),
                leftBack.getCurrentPosition(),
                rightBack.getCurrentPosition()
        );
        rightArm = hardwareMap.servo.get("right arm");
        leftArm = hardwareMap.servo.get("left arm");

//        whiteLine.enableLed(true); // mode is for close-range testing on objects that do not shine light
//        telemetry.addData("LED", true ? "On" : "Off");
//        telemetry.addData("Clear", whiteLine.alpha());
//        telemetry.addData("Red ", whiteLine.red());
//        telemetry.addData("Green", whiteLine.green());
//        telemetry.addData("Blue ", whiteLine.blue());

        ballSensor.enableLed(true); // mode is for close-range testing on objects that do not shine light
        telemetry.addData("LED", true ? "On" : "Off");
        telemetry.addData("Clear", ballSensor.alpha());
        telemetry.addData("Red ", ballSensor.red());
        telemetry.addData("Green", ballSensor.green());
        telemetry.addData("Blue ", ballSensor.blue());

        telemetry.update();
        //Vuforia - starting the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        //License Key, I already set it up I think
        parameters.vuforiaLicenseKey = "AZD9V+f/////AAAAGZDj5Sa0JkeSohNtCSdf0T94R9bz9UlzQyuCIZuJ2d1ehAEqmbPYzprSq4hmd/9XUZYT088pUIzwl79q9h2ljFjUFD5p0RHKBx+ggMJ+qgCelvbeNf7Rd771vlduzibSBN6np49m6Z31Eyk0dYFZJbpdmw4P7mQ8LaeR6UOLgmiythqcCZga9VoEHPA2e8Z9/7At1SZVPiOBkVlEKz5AGhPhL5/A/R3sb30NSaiq5yquyJ+sOWvNQ5ovdVND6OrAQrc2DdQcCDyD8JQLOiVZYCPoNohKfuZ9N2jnZRSueEH4XV6i2DOqWxfJ5vmNf6jBcrOWLROO8KEoPa2Fvibxj7lPMp4JM/nMXK7TwEopU91v";
        //Which camera we choose - I used back right now because I think the range is better, but front could work
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        //Define Pictographs as VuMarks which the Vuforia can track
//        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
//        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            relicTrackables.activate();
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                telemetry.addData("VuMark", "%s visible", vuMark); //variable vuMark --> "center", "left", "right"

            }
            else {
                telemetry.addData("VuMark", "not visible");
            }

            telemetry.update();
        }
        // Start of Autonomous!!!!                   *****************************************************************************************************************
        allianceSide("red");
        pushBall();
        senseGlyph();
        setGlyph();

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double frontLeftInches, double frontRightInches, double backLeftInches, double backRightInches,
                             double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            float hsvValues[] = {0F,0F,0F};// Stores hue and saturation values
            final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);// this may be unnecessary
            //colorSensor.enableLed(false);
            Color.RGBToHSV(ballSensor.red() * 8, ballSensor.green() * 8, ballSensor.blue() * 8, hsvValues);//turns values into simpler numbers
//            Color.RGBToHSV(whiteLine.red() * 8, whiteLine.green() * 8, whiteLine.blue() * 8, hsvValues);//turns values into simpler numbers
            //Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsvValues);//stores values

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = leftFront.getCurrentPosition() + (int) (frontLeftInches * COUNTS_PER_INCH);
            newFrontRightTarget = rightFront.getCurrentPosition() + (int) (frontRightInches * COUNTS_PER_INCH);
            newBackLeftTarget = leftFront.getCurrentPosition() + (int) (backLeftInches * COUNTS_PER_INCH);
            newBackRightTarget = rightBack.getCurrentPosition() + (int) (backRightInches * COUNTS_PER_INCH);
            leftFront.setTargetPosition(newFrontLeftTarget);
            rightFront.setTargetPosition(newFrontRightTarget);
            leftBack.setTargetPosition(newBackLeftTarget);
            rightBack.setTargetPosition(newBackRightTarget);
            // Turn On RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            leftFront.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            leftBack.setPower(Math.abs(speed));
            rightBack.setPower(Math.abs(speed));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        leftFront.getCurrentPosition(),
                        rightFront.getCurrentPosition(),
                        leftBack.getCurrentPosition(),
                        rightBack.getCurrentPosition());
                telemetry.update();
            }
            // Stop all motion;
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }

    int autDriveSpd = 50;
    //Alliance Side Method!
    boolean isBlue = false;
    int numMult = 1;
    public void allianceSide(String redBlueSide) {
        if(redBlueSide.equals("red")) {
            numMult = 1;
            isBlue = false;
        }
        else if(redBlueSide.equals("blue")) {
            numMult = -1;
            isBlue = true;
        }
        else {
            telemetry.addData("Error", "alliance side? " + redBlueSide);
            telemetry.update();
        }
    }

    //Driving Methods
    public void pushBall() {
//        whiteLine.enableLed(true);
        ballSensor.enableLed(true);
        do {
            armServo.setPosition(1);
            wait(1);
//        sense();
            if(ballSensor.red() < 200 || ballSensor.blue() < 200) {
                armServo.setPosition(0);
            }
        }while(ballSensor.red() >= 200 || ballSensor.blue() >= 200);

        //assuming that the servo is on the right side of the robot
//        sense();
        double timeToPicto = 0;
        if(isBlue == true) { //senses backwards
            if(ballSensor.blue()-50 > ballSensor.red()) { //if is blue
                driveForward(autDriveSpd,1);
                timeToPicto = .5;
            }
            else if(ballSensor.red()-50 > ballSensor.blue()) {
                driveForward(-autDriveSpd,1);
                timeToPicto = 1.5;
            }
            else {
                telemetry.addData("pushBall", "Error in push ball method (blue)");
                telemetry.update();
            }
        }
        else {
            if(ballSensor.red()-50 > ballSensor.blue()) { //if is red
                driveForward(autDriveSpd,1);
                timeToPicto = .5;
            }
            else if(ballSensor.blue()-50 > ballSensor.red()) {
                driveForward(-autDriveSpd,1);
                timeToPicto = 1.5;
            }
            else {
                telemetry.addData("pushBall", "Error in push ball method (red)");
                telemetry.update();
            }
        }
        armServo.setPosition(0);
//        whiteLine.enableLed(false);
        ballSensor.enableLed(false);
        wait(1);
        driveForward(60, timeToPicto);
    }

    public void senseGlyph() {
//        rotateRight(numMult * autDriveSpd,1);  //maybe be unnecessary depending on phone placement
        relicTrackables.activate(); //sense()
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate); //camera
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            if(vuMark.equals("left")) {
                pictoRow = 1; //just becuase I can
            }
            if(vuMark.equals("center")) {
                pictoRow = 2; //just becuase I can
            }
            if(vuMark.equals("right")) {
                pictoRow = 3; //just becuase I can
            }
        }
    }

    int pictoRow = 0; //for knowing which row the pictograph shows for the glyph
    /*
    1 = left
    2 = middle
    3 = right
    */
    public void setGlyph() {
        double timeToGlyph = 0;
        if(isBlue == true) { //blue
            rotateRight(autDriveSpd,1);
            if(pictoRow == 1) {
                timeToGlyph = -1;
            }
            else if(pictoRow == 2) {
                timeToGlyph = -1.5;
            }
            else if(pictoRow == 3) {
                timeToGlyph = -2;
            }
            timeToGlyph *= numMult;
        }
        else if(isBlue == false) { //red
            rotateLeft(autDriveSpd, 1);
            if(pictoRow == 1) {
                timeToGlyph = 2;
            }
            else if(pictoRow == 2) {
                timeToGlyph = 1.5;
            }
            else if(pictoRow == 3) {
                timeToGlyph = 1;
            }
//            timeToGlyph *= numMult;
        }
        else {
            telemetry.addData("getGlyph", "Error in getting glyph: " + pictoRow);
            telemetry.update();
        }

        driveForward((int)(autDriveSpd * .85), timeToGlyph);
        rotateRight(numMult * autDriveSpd, 1);
        openArm(1); //open
        driveBackward((int)(autDriveSpd * .8), .5); //deposites and pushes in block
        openArm(-1); //close
        driveForward(autDriveSpd, .8);
    }

    //Extra Methods
    public void driveForward(int x, double time){
        encoderDrive(DRIVE_SPEED,  x,  x, x, x, time);  // S1: Forward 47 Inches with 5 Sec timeout
    }
    public void driveBackward(int x, double time){
        encoderDrive(DRIVE_SPEED,  -x,  -x, -x, -x, time);  // S1: Forward 47 Inches with 5 Sec timeout
    }
    public void strafeLeft(int x, double time){
        encoderDrive(STRAFE_SPEED,  -x,  x, x, -x, time);  // S1: Forward 47 Inches with 5 Sec timeout
    }
    public void strafeRight(int x, double time){
        encoderDrive(STRAFE_SPEED,  x,  -x, -x, x, time);  // S1: Forward 47 Inches with 5 Sec timeout
    }
    public void rotateRight(int x, double time){
        encoderDrive(TURN_SPEED,  x,  -x, x, -x, time);  // S1: Forward 47 Inches with 5 Sec timeout
    }
    public void rotateLeft(int x, double time){
        encoderDrive(TURN_SPEED,  -x,  x, -x, x, time);  // S1: Forward 47 Inches with 5 Sec timeout
    }
    public void openArm(int pos) {
        rightArm.setPosition(pos);
        leftArm.setPosition(-pos);
    }

    public void wait(int time){
        try{
            Thread.sleep(time * 1000); //seconds
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
