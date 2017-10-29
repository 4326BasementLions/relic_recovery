package org.firstinspires.ftc.teamcode.SensorBNO055IMU;
//package org.firstinspires.ftc.robotcontroller.external.samples;

        import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
        import com.qualcomm.hardware.bosch.BNO055IMU;
        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.util.ReadWriteFile;

        import org.firstinspires.ftc.robotcore.external.Func;
        import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
        import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
        import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
        import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

        import java.io.File;
        import java.util.Locale;

/**
 * Created by Libby on 10/27/17.
 */
@TeleOp(name = "Sensor: BNO055 IMU Calibration", group = "Sensor")
public class SensorBNO055IMUCalibration extends LinearOpMode{
    BNO055IMU imu; //Variable name for the sensor itself
    Orientation angles;//Variable name for the angle at which the robot is at to update telemetry
    Acceleration gravity;//again used to update telemetry
    @Override
    public void runOpMode() {
        telemetry.log().setCapacity(12);
       /*telemetry.log().add("");
        telemetry.log().add("Please refer to the calibration instructions");
        telemetry.log().add("contained in the Adafruit IMU calibration");
        telemetry.log().add("sample opmode.");
        telemetry.log().add("");*/
        telemetry.log().add("When sufficient calibration has been reached,");
        telemetry.log().add("press the 'A' button to write the current");
        telemetry.log().add("calibration data to a file.");
        telemetry.log().add(""); //Instructions on how to calibrate

        //IMU should be on the 12C section in calibration (automatically there I believe)

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(); // what it can see
        parameters.loggingEnabled = true; //allows parameters to be recorded (in telemetry or sensor?)
        parameters.loggingTag     = "IMU";  //name in logs
        imu = hardwareMap.get(BNO055IMU.class, "imu"); //name for config
        imu.initialize(parameters);//sets up parameters
        composeTelemetry();
        //telemetry.update();//put this instead of above but realized above defined below
        telemetry.log().add("Waiting for start...");

        while (!isStarted()) {
            telemetry.update();
            idle();
        }//When waiting for play, the telemetry waits

        telemetry.log().add("Robot has started ^^");//let's us know play has been pushed


        while (opModeIsActive()) {
            if (gamepad1.a) {
                BNO055IMU.CalibrationData calibrationData = imu.readCalibrationData();//gets calibration data
                /*The calibration from here will be saved to a file of your choosing.  This is so we
                * only have to calibrate once.  Make sure the file name is the same when you conigure for
                 * an opmode! */
                String filename = "AdafruitIMUCalibration.json";
                File file = AppUtil.getInstance().getSettingsFile(filename);
                ReadWriteFile.writeFile(file, calibrationData.serialize());
                telemetry.log().add("saved to '%s'", filename);

                while (gamepad1.a) { //press button to start config, release to confirm
                    telemetry.update();
                    idle();

            }

        }
            telemetry.update();

    }
}
    void composeTelemetry() {
        //data grabbed and displayed
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);//tells you where robot is
            gravity  = imu.getGravity();
        }
        });
        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle); //tells degrees of first angle(z axis?)
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);//tells degrees of second angle(y axis?)
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);//tells degrees of third angle(x axis)
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();//tells gravity value
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));  //orientation based on axes and gravity value
                    }
                });
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
