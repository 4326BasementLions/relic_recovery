/**
 * Created by shreysahgal on 12/3/17.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import java.lang.*;

@TeleOp(name = "hybrid legit legit moov mo2", group = "Tele Op")
public class HybridTeleOp extends OpMode
{
    // wheel motors
    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;

    // intake motors
    DcMotor leftIntake;
    DcMotor rightIntake;
    Servo right;
    Servo left;
    DcMotor rightLift;
    DcMotor leftLift;

    // int that will be used for logic
    int intakeDir = 0;

    double rightPos = 0.5;
    double leftPos = 0.5;

    double rightUpperLimit = 0.85;
    double rightLowerLimit = /*0.22*/ .02;
    double leftLowerLimit = 0.15;
    double leftUpperLimit = 0.78;



    public void init()
    {
        frontRight = hardwareMap.dcMotor.get("frontRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        leftIntake = hardwareMap.dcMotor.get("leftIntake");
        rightIntake = hardwareMap.dcMotor.get("rightIntake");

        right = hardwareMap.servo.get("right");
        left = hardwareMap.servo.get("left");

        rightLift = hardwareMap.dcMotor.get("rightLift");
        leftLift = hardwareMap.dcMotor.get("leftLift");

        right.setPosition(rightPos);
        left.setPosition(leftPos);
    }

    public void loop()
    {
        //telemetry.addData("Intake speed", intakeSpeed);
        float drive = -gamepad1.left_stick_y;
        float turn = gamepad1.right_stick_x;
        float strafe = -gamepad1.left_stick_x;

        double flDrive = Range.clip(drive + turn - strafe, -1.0, 1.0);
        double blDrive = Range.clip(drive + turn + strafe, -1.0, 1.0);
        double frDrive = Range.clip(drive - turn + strafe, -1.0, 1.0);
        double brDrive = Range.clip(drive - turn - strafe, -1.0, 1.0);

        frontRight.setPower(frDrive);
        backRight.setPower(brDrive);
        frontLeft.setPower(flDrive);
        backLeft.setPower(blDrive);

        if(gamepad1.b) {
            rightIntake.setPower(0);
            leftIntake.setPower(0);
        }
        if(gamepad1.left_trigger > 0)
        {
            leftIntake.setPower(-gamepad1.left_trigger);
            rightIntake.setPower(gamepad1.left_trigger);
        }
        else if(gamepad1.right_trigger > 0)
        {
            leftIntake.setPower(gamepad1.right_trigger);
            rightIntake.setPower(-gamepad1.right_trigger);
        }
        else if(gamepad1.a)
        {
            leftIntake.setPower(0);
            rightIntake.setPower(0);
        }

        /*if(gamepad2.right_stick_y > 0)
        {
            rightPos += 0.001;
            leftPos -= 0.001;
        }
        else if(gamepad2.right_stick_y < 0)
        {
            rightPos -= 0.001;
            leftPos += 0.001;
        }*/

        /*if(false) // down
        {
            right.setPosition(0.883);
            left.setPosition(0.117);
        }
        if(gamepad2.b) // level
        {
            right.setPosition(0.656);
            left.setPosition(0.344);
        }
        if(gamepad2.a) // up
        {
            right.setPosition(0);
            left.setPosition(1);

        }

        gamepad2.right_stick_y > 0
        gamepad2.right_stick_y < 0
        */

        if(gamepad2.a && rightPos > rightLowerLimit)
        {
            rightPos -= 0.005;
            leftPos += 0.005;
        }
        else if(gamepad2.b && rightPos < rightUpperLimit)
        {
            rightPos += 0.005;
            leftPos -= 0.005;
        }

        /*if(gamepad2.a)
        {
            rightPos = 0;
            leftPos = 1;
        }*/

        right.setPosition(rightPos);
        left.setPosition(leftPos);

        rightLift.setPower(gamepad2.left_stick_y);
        leftLift.setPower(-gamepad2.left_stick_y);

        telemetry.addData("right", rightPos);
        telemetry.addData("left", leftPos);
        telemetry.update();

    }
}