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

@TeleOp(name = "Hybrid Tele Op", group = "Tele Op")
public class HybridTeleOp extends OpMode
{

    // wheel motors
    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;

    // intakes
    DcMotor leftIntake;
    DcMotor rightIntake;

    // swinging arm
    DcMotor swing;

    // servo arms
    Servo rightArm;
    Servo leftArm;

    // lifters
    DcMotor rightLifter;
    DcMotor leftLifter;

    public void init()
    {
        frontRight = hardwareMap.dcMotor.get("frontRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        rightIntake = hardwareMap.dcMotor.get("rightIntake");
        leftIntake = hardwareMap.dcMotor.get("leftIntake");

        swing = hardwareMap.dcMotor.get("swing");

        rightArm = hardwareMap.servo.get("rightArm");
        leftArm = hardwareMap.servo.get("leftArm");

        rightLifter = hardwareMap.dcMotor.get("rightLifter");
        leftLifter = hardwareMap.dcMotor.get("leftLifter");

        rightArm.setPosition(-1);
        leftArm.setPosition(1);
    }

    public void loop()
    {
        // driving controls
        float gp1LeftStickY = gamepad1.left_stick_y;
        float gp1LeftStickX = gamepad1.left_stick_x;
        float gp1RightStickX = gamepad1.right_stick_x;

        float drive = -gp1LeftStickY;
        float turn = gp1RightStickX;
        float strafe = -gp1LeftStickX;

        double frDrive = Range.clip(drive - turn + strafe, -1.0, 1.0);
        double flDrive = Range.clip(drive + turn - strafe, -1.0, 1.0);
        double brDrive = Range.clip(drive - turn - strafe, -1.0, 1.0);
        double blDrive = Range.clip(drive + turn + strafe, -1.0, 1.0);

        frontRight.setPower(frDrive);
        frontLeft.setPower(flDrive);
        backRight.setPower(brDrive);
        backLeft.setPower(blDrive);

        // intake
        boolean gp2a = gamepad2.a;


        if(gp2a)
        {
            rightLifter.setPower(0.2);
            leftLifter.setPower(-0.2);

            rightArm.setPosition(-1);
            leftArm.setPosition(1);

            rightLifter.setPower(-0.2);
            leftLifter.setPower(0.2);
        }

        // swinging arm
        double gp2LeftY = gamepad2.left_stick_y;

        swing.setPower(gp2LeftY);

        // release glyph
        boolean gp2b = gamepad2.b;

        if(gp2b)
        {
            rightArm.setPosition(1);
            leftArm.setPosition(-1);
        }
    }
}
