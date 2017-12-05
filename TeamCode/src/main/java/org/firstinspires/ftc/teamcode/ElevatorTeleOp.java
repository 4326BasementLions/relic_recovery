package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "Full Tele Op", group = "Tele Op")
public class ElevatorTeleOp extends OpMode {

    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;

    DcMotor lift;

    Servo rightArm;
    Servo leftArm;

    @Override
    public void init()
    {
        frontRight = hardwareMap.dcMotor.get("frontRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");

        lift = hardwareMap.dcMotor.get("lift");

        rightArm = hardwareMap.servo.get("rightArm");
        leftArm = hardwareMap.servo.get("leftArm");

    }

    public float rightPos = 1;
    public float leftPos = 0;
    
    @Override
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

        // lift control
        float gp2LeftY = gamepad2.left_stick_y;

        lift.setPower(gp2LeftY*0.6);

        // glypht arm control
        float gp2RightY = gamepad2.right_stick_y;

        if((rightPos+.025 <= 1) && (leftPos-.025 >= 0)) {
            if(gp2RightY > 0) // open
            {
                rightPos += 0.025;
                leftPos -= 0.025;
            }
        }
        if((rightPos-.025 >= 0) && (leftPos+.025) <=1) {
            if(gp2RightY < 0) // close
            {
                rightPos -= 0.025;
                leftPos += 0.025;
            }
        }
        else if(gamepad2.x) {
            rightPos = 0;
            leftPos = 0;
        }

        rightArm.setPosition(rightPos);
        leftArm.setPosition(leftPos);

    }

}
