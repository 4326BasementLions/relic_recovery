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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by Libby on 12/3/17.
 */
@Autonomous(name="SkellyOpElevator", group="Autonomous")
public class AutoSkellyOpElevator extends LinearOpMode {
    private ElapsedTime     runtime = new ElapsedTime();

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    DcMotor lifter;

    Servo leftArm;
    Servo rightArm;

    ColorSensor ballSensor;
    //servo color mechanism
    Servo colorNumber1;
    Servo colorNumber2;

    double strafeTime;



    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;


    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */

    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode(){
        //set up
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();


        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");



        lifter = hardwareMap.dcMotor.get("lifter");


        ballSensor = hardwareMap.colorSensor.get("ballSensor");

        colorNumber1 = hardwareMap.servo.get("jewelArm");
        colorNumber2 = hardwareMap.servo.get("jewelpush");

        leftArm = hardwareMap.servo.get("leftArm");
        rightArm = hardwareMap.servo.get("rightArm");

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        colorNumber1.setPosition(0);
        colorNumber2.setPosition(0);

        leftArm.setPosition(.5);
        rightArm.setPosition(.5); //Whichever value will hold a glyph without burning out the servo

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


        waitForStart();

        ballSensor.enableLed(true);

        boolean detected = false;
        while(detected == false ) {
            colorNumber1.setPosition(0);
            if (ballSensor.red() > ballSensor.blue()) {
                telemetry.addData("color", "red");
                telemetry.update();
                colorNumber1.setPosition(.5);
                colorNumber2.setPosition(1);
                colorNumber1.setPosition(0);
                //armServo.setPosition(1);
                sleep(1000);
                detected = true;
            } else if (ballSensor.red() < ballSensor.blue()) {
                telemetry.addData("color", "blue");
                telemetry.update();
                colorNumber1.setPosition(.5);
                colorNumber2.setPosition(0);
                colorNumber1.setPosition(0);
                //armServo.setPosition(1);
                sleep(1000);

                strafeLeft(0.5, 0.2);
                detected = true;
            }

            strafeLeft(0.2, 0.3);

            sleep(1000);
            relicTrackables.activate();
            boolean detectedPicto = false;
            RelicRecoveryVuMark vuMark;
            while(detectedPicto == false) {
                vuMark = RelicRecoveryVuMark.from(relicTemplate);
                telemetry.addData("VuMark", "%s visible", vuMark);
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                    if(vuMark==RelicRecoveryVuMark.RIGHT){
                        strafeTime=.25;
                    }
                    else if(vuMark==RelicRecoveryVuMark.CENTER){
                        strafeTime=.4;
                    }
                    else if(vuMark==RelicRecoveryVuMark.LEFT){
                        strafeTime=.55;
                    }

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
//                telemetry.addData("VuMark", "%s visible", vuMark);

                    detectedPicto  = true;
                    telemetry.addData("Vumark", vuMark);
                    strafeLeft(0.2, strafeTime);

                    driveForward(2,1);

                    leftArm.setPosition(0);
                    rightArm.setPosition(1);

                    driveBackward(2,1);

                    stopRobot();



                }
                else {
                    telemetry.addData("VuMark", "not visible");
                    strafeLeft(0.15, 0.25);
                }

                telemetry.update();
            }

        }
    }
    public void encoderDrive(double speed,
                             double leftbackInches, double rightbackInches, double leftfrontInches, double rightFrontInches,
                             double timeoutS) {
        int newbackLeftTarget;
        int newbackRightTarget;
        int newfrontLeftTarget;
        int newfrontRightTarget;

        int optimalAngle = 90;
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
    public void turnLeft(double power, double time) {
        encoderDrive(DRIVE_SPEED, power, power, power, power, time);
    }
    public void turnRight(double power, double time) {
        encoderDrive(DRIVE_SPEED, -power, -power, -power, -power, time);
    }
    public void strafeLeft(double power, double time){
        encoderDrive(DRIVE_SPEED,power,-power,-power,power,time);
    }
    public void strafeRight(double power, double time) {
        encoderDrive(DRIVE_SPEED, -power, power, power, -power, time);
    }
    public void driveForward(double power, double time) {
        encoderDrive(DRIVE_SPEED, -power, power, -power, power, time);
    }
    public void driveBackward(double power, double time) {
        encoderDrive(DRIVE_SPEED, power, -power, power, -power, time);
    }
    public void stopRobot() {
        encoderDrive(0, 0, 0, 0, 0, 0.1);
    }
}


