package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.net.CookieStore;
import java.util.ArrayList;

/**
 * RobotController controls all hardware on the robot
 * and abstracts all computation done by the robot
 */

public class RobotController {
    // Sensors
    public ColorSensor testColorSensor;
    public DigitalChannel testTouchSensor;

    private LinearOpMode context;
    private HardwareMap hmap;

    // Motors
    public DcMotor frontRight;
    public DcMotor frontLeft;
    public DcMotor backRight;
    public DcMotor backLeft;

    // Servos
    public CRServo testServo;

    public RobotController(LinearOpMode context) {
        this.context = context;

        hmap = context.hardwareMap;

        // Setting up motors
        frontRight = hmap.dcMotor.get("front_right");
        frontLeft = hmap.dcMotor.get("front_left");
        backRight = hmap.dcMotor.get("back_right");
        backLeft = hmap.dcMotor.get("back_left");

        // Sensors
        testColorSensor = hmap.colorSensor.get("color");
        testTouchSensor = hmap.digitalChannel.get("touch");
        testColorSensor.enableLed(true);
        testTouchSensor.setMode(DigitalChannel.Mode.INPUT);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Servos
        testServo = hmap.crservo.get("servo");
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

    public int[] getARGB(ColorSensor sensor) {
        int[] argb = {0,0,0,0};
        int color = sensor.argb();
        argb[0] = color >> 24 & (0b11111111);
        argb[1] = (color << 8) >> 24 & (0b11111111);
        argb[2] = (color << 16) >> 24 & (0b11111111);
        argb[3] = color & (0b11111111);

        return argb;
    }

    /**
     * Gets color value.
     *
     * @param tries number of trials
     * @return gold or white
     */
    public SensorColor getColor(ColorSensor sensor) {
        float[] hslValue = {0, 0, 0};

        // Convert to HSV
        Color.RGBToHSV((int)sensor.red() * 255, (int)sensor.green() * 255, (int)sensor.blue() * 255, hslValue);

        SensorColor output = SensorColor.COLOR_EMPTY;

        if (hslValue[0] < 90 && hslValue[2] > 500) {
            output = SensorColor.COLOR_GOLD;
        }else if (hslValue[0] > 110 && hslValue[2] > 500) {
            output = SensorColor.COLOR_WHITE;
        } else {
            output = SensorColor.COLOR_EMPTY;
        }

        return output;
    }

    /**
     * Gets state of touch sensor.
     */
    public boolean isTouchSensorPressed(DigitalChannel touchSensor) {
        return !touchSensor.getState();
    }

    /**
     * Powers a CRServo.
     */
    public void powerCRServo(CRServo servo, double pow) {
        servo.setDirection(CRServo.Direction.FORWARD);
        servo.setPower(pow);
    }
}