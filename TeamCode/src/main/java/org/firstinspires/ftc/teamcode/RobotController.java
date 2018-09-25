package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * RobotController controls all hardware on the robot
 * and abstracts all computation done by the robot
 */

public class RobotController {
    // color sensor
    public ColorSensor colorSensor;
    public int[] WHITE_RGB = {255, 255, 255};
    public int[] GOLD_RGB = {255, 215, 0};

    private LinearOpMode context;
    private HardwareMap hmap;

    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;

    public RobotController(LinearOpMode context) {
        this.context = context;

        hmap = context.hardwareMap;

        // setting up motors
        frontRight = hmap.dcMotor.get("front_right");
        frontLeft = hmap.dcMotor.get("front_left");
        backRight = hmap.dcMotor.get("back_right");
        backLeft = hmap.dcMotor.get("back_left");

        // sensors

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Rotates the robot.
     *
     * @param angle angle in radians
     */
    public void rotateAngle(double angle) {
    }

    /**
     * Moves the robot.
     *
     * @param distance distance in meters
     */
    public void moveDistance(double distance, int direction) {
    }

    /**
     * Gets color value.
     *
     * @param tries number of trials
     * @return gold or white
     */
    public Constant getRawColor(ColorSensor sensor, int tries) {
        int threshold = (int) 255 / 2;
        int whiteCount = 0;
        int goldCount = 0;

        for (int i = 0; i < tries; i++)
        {
            int val = sensor.blue();

            if (val > threshold) { whiteCount++; }
            else { goldCount++; }
        }

        if (whiteCount > goldCount) {
            return Constant.COLOR_WHITE;
        } else {
            return Constant.COLOR_GOLD;
        }
    }
}