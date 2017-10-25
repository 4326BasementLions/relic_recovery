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
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY’S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
 * Use Android Studios to Copy this Class, and Paste it into your team’s code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Auto Drive By Encoder1.5.1", group="Autonomous")

public class auto extends LinearOpMode {

    /* Declare OpMode members. */

    private ElapsedTime     runtime = new ElapsedTime();

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    ColorSensor ballSensor;
    Servo armServo;


    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();


        frontLeft = hardwareMap.dcMotor.get("left front");
        frontRight = hardwareMap.dcMotor.get("right front");
        backLeft = hardwareMap.dcMotor.get("left back");
        backRight = hardwareMap.dcMotor.get("right back");

        armServo = hardwareMap.servo.get("armServo");
        ballSensor = hardwareMap.colorSensor.get("ballSensor");




        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                backLeft.getCurrentPosition(),
                backRight.getCurrentPosition(),
                frontLeft.getCurrentPosition(),
                frontRight.getCurrentPosition());

        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AZD9V+f/////AAAAGZDj5Sa0JkeSohNtCSdf0T94R9bz9UlzQyuCIZuJ2d1ehAEqmbPYzprSq4hmd/9XUZYT088pUIzwl79q9h2ljFjUFD5p0RHKBx+ggMJ+qgCelvbeNf7Rd771vlduzibSBN6np49m6Z31Eyk0dYFZJbpdmw4P7mQ8LaeR6UOLgmiythqcCZga9VoEHPA2e8Z9/7At1SZVPiOBkVlEKz5AGhPhL5/A/R3sb30NSaiq5yquyJ+sOWvNQ5ovdVND6OrAQrc2DdQcCDyD8JQLOiVZYCPoNohKfuZ9N2jnZRSueEH4XV6i2DOqWxfJ5vmNf6jBcrOWLROO8KEoPa2Fvibxj7lPMp4JM/nMXK7TwEopU91v";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        //encoderDrive(DRIVE_SPEED,  -1,  -1, 1, 1, 1.0);  // S1: Right X Inches with 5 Sec timeout
        //encoderDrive(TURN_SPEED,   .3, -.3, .3, -.3, 4.0);  // S2:
        //encoderDrive(DRIVE_SPEED, .1, .1, -.1, -.1, 4.0);  // S3:

          // pause for servos to move RED TEAM
        ballSensor.enableLed(true);
        boolean detected = false;
        while(detected == false ) {
            armServo.setPosition(0.05);
            if(ballSensor.red() > ballSensor.blue()) {
                telemetry.addData("color", "red");
                telemetry.update();
                turnLeft(1, 0.1);
                armServo.setPosition(1);
                sleep(1000);
                turnRight(1,0.1);
                detected = true;
            } else if(ballSensor.red() < ballSensor.blue()) {
                telemetry.addData("color", "blue");
                telemetry.update();
                turnRight(1, 0.1);
                armServo.setPosition(1);
                sleep(1000);
                turnLeft(1,0.1);
                detected = true;
            }
        }

        sleep(1000);
        boolean detectedPicto = false;
        while(detectedPicto == false) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            telemetry.addData("VuMark", "%s visible", vuMark);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
//                telemetry.addData("VuMark", "%s visible", vuMark);
                detectedPicto  = true;


            }
            else {
//                telemetry.addData("VuMark", "not visible");
            }

            telemetry.update();
        }
    }




//Strafe Right -x -x, x, x


    //back x -x -x x
    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftbackInches, double rightbackInches, double leftfrontInches, double rightFrontInches,
                             double timeoutS) {
        int newbackLeftTarget;
        int newbackRightTarget;
        int newfrontLeftTarget;
        int newfrontRightTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newbackLeftTarget = backLeft.getCurrentPosition() + (int)(leftbackInches * COUNTS_PER_INCH);
            newbackRightTarget = backRight.getCurrentPosition() + (int)(rightbackInches * COUNTS_PER_INCH);
            newfrontLeftTarget = frontLeft.getCurrentPosition() + (int)(leftfrontInches * COUNTS_PER_INCH);
            newfrontRightTarget = frontRight.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
            backLeft.setTargetPosition(newbackLeftTarget);
            backRight.setTargetPosition(newbackRightTarget);
            frontLeft.setTargetPosition(newfrontLeftTarget);
            frontRight.setTargetPosition(newfrontRightTarget);

            // Turn On RUN_TO_POSITION
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            backLeft.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));
            frontLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is “safer” in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (backLeft.isBusy() && backRight.isBusy() && frontLeft.isBusy() && frontRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newbackLeftTarget,  newbackRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                backLeft.getCurrentPosition(),
                        backRight.getCurrentPosition(),
                        frontLeft.getCurrentPosition(),
                        frontRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            backLeft.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);
            // Turn off RUN_TO_POSITION
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }

    public void turnLeft(int power, double time) {
        encoderDrive(DRIVE_SPEED, -power, power, -power, power, time);

    }
    public void turnRight(int power, double time) {
        encoderDrive(DRIVE_SPEED, power, -power, power, -power, time);

    }
}
