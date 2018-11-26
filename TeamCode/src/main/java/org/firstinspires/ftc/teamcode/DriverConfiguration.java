package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.lang.reflect.Array;
import java.util.Arrays;

/**
 * This class takes care of all possible polarities of the robot
 * and speeds up the repetitive nature of driver op modes.
 */

public class DriverConfiguration {
    // Motors
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;

    private double maxPower = 0.0;
    private double[] power = {0, 0, 0, 0};
    private int[] forwardVector = {0, 0, 0, 0};

    /**
     * @param forwardVector in the form frontRight, frontLeft, backRight, backLeft
     */
    public DriverConfiguration(DcMotor frontRight, DcMotor frontLeft, DcMotor backRight, DcMotor backLeft, int[] forwardVector, double maxPower) {
        // Setting up motors
        this.frontRight = frontRight;
        this.frontLeft = frontLeft;
        this.backRight = backRight;
        this.backLeft = backLeft;

        this.forwardVector = forwardVector;

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.maxPower = maxPower;
    }

    public void addTransPower(double pow) {
        power[0] += pow;
        power[1] += pow;
        power[2] += pow;
        power[3] += pow;
    }

    public void addTransPower(double[] pow) {
        power[0] += pow[0];
        power[1] += pow[1];
        power[2] += pow[2];
        power[3] += pow[3];
    }

    public void addRotPower(double pow) {
        power[0] += pow;
        power[1] += -1 * pow;
        power[2] += pow;
        power[3] += -1 * pow;
    }

    public void addRotPower(double[] pow) {
        power[0] += pow[0];
        power[1] += -1 * pow[1];
        power[2] += pow[2];
        power[3] += -1 * pow[3];
    }

    public void zeroPower() {
        power[0] = 0;
        power[1] = 0;
        power[2] = 0;
        power[3] = 0;
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

    /**
     * Scales all the power values such that the highest power is the max power of the configuration if it exceeds the limit.
     */
    public void scalePower() {
        // First, we find the highest powered motor.
        double[] absolutePower = new double[]{Math.abs(power[0]), Math.abs(power[1]), Math.abs(power[2]), Math.abs(power[3])};
        Arrays.sort(absolutePower);

        if (absolutePower[3] > maxPower) {
            double mag = Math.sqrt(Math.pow(power[0], 2) + Math.pow(power[1], 2) + Math.pow(power[2], 2) + Math.pow(power[3], 2));
            power[0] /= mag;
            power[1] /= mag;
            power[2] /= mag;
            power[3] /= mag;

            // Now, we find the amount to scale.
            double scalar = absolutePower[3] / maxPower;

            // Scale.
            power[0] *= scalar;
            power[1] *= scalar;
            power[2] *= scalar;
            power[3] *= scalar;
        }
    }

    public void startMotors() {
        scalePower();

        frontRight.setPower(forwardVector[0] * power[0]);
        frontLeft.setPower(forwardVector[1] * power[1]);
        backRight.setPower(forwardVector[2] * power[2]);
        backLeft.setPower(forwardVector[3] * power[3]);
    }

    public void stopMotors() {
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    public double[] getPower() {
        return new double[]{power[0], power[1], power[2], power[3]};
    }
}