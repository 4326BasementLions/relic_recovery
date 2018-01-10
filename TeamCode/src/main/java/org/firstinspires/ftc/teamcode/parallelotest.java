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

@TeleOp(name = "parallelotest", group = "TeleOp")
public class parallelotest extends OpMode
{
    DcMotor lift;
    Servo left;
    Servo right;

    public float rightPos = 1;
    public float leftPos = 0;

    public void init()
    {
        lift = hardwareMap.dcMotor.get("lift");
        left = hardwareMap.servo.get("left");
        right = hardwareMap.servo.get("right");
    }

    public void loop()
    {
        float gp1LeftStickY = gamepad1.left_stick_y;
        float gp1RightStickY = gamepad1.right_stick_y;

        lift.setPower(gp1LeftStickY);

        if(gp1RightStickY > 0)
        {
            rightPos++;
            leftPos--;
        }
        if(gp1RightStickY < 0)
        {
            rightPos--;
            leftPos++;
        }

        right.setPosition(rightPos);
        left.setPosition(leftPos);

    }
}
