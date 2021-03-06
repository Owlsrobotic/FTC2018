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
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Motion Calibrator", group="Linear Opmode")
public class MovementCalibrator extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    double[] fowardVector = {-1.0,1.0,-1.0,1.0};
    double[] rightRotateVector = {-1.0, -1.0, -1.0, -1.0};

    private DcMotor backleft = null;
    private DcMotor backright = null;
    private DcMotor frontleft = null;
    private DcMotor frontright = null;

    BNO055IMU imu;

    double     COUNTS_PER_MOTOR_REV    = 2240 ;
    //Good Number: 0.51625
    double     TRANSLATION_FACTOR    = 2.0 ;
    double     WHEEL_DIAMETER_M   = 0.1016;
    double     COUNTS_PER_M         = (COUNTS_PER_MOTOR_REV * TRANSLATION_FACTOR) /
            (WHEEL_DIAMETER_M * 3.1415);

    double timeOutMillis = 30000;
    double threshold = 0.2;
    //Good Number: 0.04456049
    double pValue = 1.0/25.0;
    double iValue = 1.0/10.0;
    double dValue = 1.0/210.0;

    //Calibration Modes
    double pValueCalibrate = 25.0;
    double iValueCalibrate = 10.0;
    double dValueCalibrate = 210.0;
    double windupThreshold = 5.0;

    double translateFactorHigh = 4.0;
    double translateFactorLow = 0.0;

    int TRANSLATION_MODE = 0;
    int PVALUE_MODE = 1;
    int IVALUE_MODE = 2;
    int DVALUE_MODE = 3;
    int CURRENT_MODE = TRANSLATION_MODE;

    double angleChange = 90.0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Motor Set Up
        backleft = hardwareMap.dcMotor.get("back_left");
        backright = hardwareMap.dcMotor.get("back_right");
        frontleft = hardwareMap.dcMotor.get("front_left");
        frontright = hardwareMap.dcMotor.get("front_right");

        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //IMU Setup
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "IMUCalibration.json";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Controls: ", "DPAD_UP => Translation factor calibrate ... DPAD_LEFT => PVALUE calibrate ... RB => Too high ... LB => Too low");
            telemetry.addData("Controls: ", "DPAD_DOWN => IVALUE calibrate ... DPAD_RIGHT => DVALUE calibrate ... RB => Too high ... LB => Too low");
            telemetry.addData("Translation Factor: ", TRANSLATION_FACTOR);
            telemetry.addData("P VALUE CALIBRATE: ", pValueCalibrate);
            telemetry.addData("I VALUE CALIBRATE: ", iValueCalibrate);
            telemetry.addData("D VALUE CALIBRATE: ", dValueCalibrate);
            telemetry.addData("CURRENT MODE: ", CURRENT_MODE);
            telemetry.addData("ANGLE CHANGE: ", angleChange);

            if (gamepad1.dpad_up) {
                CURRENT_MODE = TRANSLATION_MODE;
            }
            if (gamepad1.dpad_left) {
                CURRENT_MODE = PVALUE_MODE;
            }
            if (gamepad1.dpad_down) {
                CURRENT_MODE = IVALUE_MODE;
            }
            if (gamepad1.dpad_right) {
                CURRENT_MODE = DVALUE_MODE;
            }
            if (gamepad1.right_trigger > 0.1) {
                angleChange += 1.0;
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            if (gamepad1.left_trigger > 0.1) {
                angleChange -= 1.0;
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

            //Current value is too high
            if (gamepad1.right_bumper) {
                if (CURRENT_MODE == TRANSLATION_MODE) {
                    translateFactorHigh = (translateFactorHigh + translateFactorLow) / 2;
                    TRANSLATION_FACTOR = (translateFactorHigh + translateFactorLow) / 2;
                    COUNTS_PER_M = (COUNTS_PER_MOTOR_REV * TRANSLATION_FACTOR) / (WHEEL_DIAMETER_M * 3.1415);
                }
                if (CURRENT_MODE == PVALUE_MODE) {
                    pValueCalibrate += 1.0;
                    pValue = 1.0/pValueCalibrate;
                }
                if (CURRENT_MODE == IVALUE_MODE) {
                    iValueCalibrate += 1.0;
                    iValue = 1.0/iValueCalibrate;
                }
                if (CURRENT_MODE == DVALUE_MODE) {
                    dValueCalibrate += 1.0;
                    dValue = 1.0/dValueCalibrate;
                }
                try {
                    Thread.sleep(250);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

            //Current value is too low
            if (gamepad1.left_bumper) {
                if (CURRENT_MODE == TRANSLATION_MODE) {
                    translateFactorLow = (translateFactorHigh + translateFactorLow) / 2;
                    TRANSLATION_FACTOR = (translateFactorHigh + translateFactorLow) / 2;
                    COUNTS_PER_M = (COUNTS_PER_MOTOR_REV * TRANSLATION_FACTOR) / (WHEEL_DIAMETER_M * 3.1415);
                }
                if (CURRENT_MODE == PVALUE_MODE) {
                    pValueCalibrate -= 1.0;
                    pValue = 1.0/pValueCalibrate;
                }
                if (CURRENT_MODE == IVALUE_MODE) {
                    iValueCalibrate -= 1.0;
                    iValue = 1.0/iValueCalibrate;
                }
                if (CURRENT_MODE == DVALUE_MODE) {
                    dValueCalibrate -= 1.0;
                    dValue = 1.0/dValueCalibrate;
                }
                try {
                    Thread.sleep(250);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

            if(gamepad1.a) {
                backleft.setPower(fowardVector[0]);
                backright.setPower(fowardVector[1]);
                frontleft.setPower(fowardVector[2]);
                frontright.setPower(fowardVector[3]);
            }

            if(gamepad1.b) {
                double distance = 1.0; //in meters
                double power = 1.0;
                int distanceInTicks = (int)(distance * COUNTS_PER_M);

                int backleftTargetPos = backleft.getCurrentPosition() + (int)(fowardVector[0] * distanceInTicks);
                int backrightTargetPos = backright.getCurrentPosition() + (int)(fowardVector[1] * distanceInTicks);
                int frontleftTargetPos = frontleft.getCurrentPosition() + (int)(fowardVector[2] * distanceInTicks);
                int frontrightTargetPos = frontright.getCurrentPosition() + (int)(fowardVector[3] * distanceInTicks);

                backleft.setTargetPosition(backleftTargetPos);
                backright.setTargetPosition(backrightTargetPos);
                frontleft.setTargetPosition(frontleftTargetPos);
                frontright.setTargetPosition(frontrightTargetPos);

                backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                backleft.setPower(power);
                backright.setPower(power);
                frontleft.setPower(power);
                frontright.setPower(power);

                while (backleft.isBusy() && backright.isBusy() && frontleft.isBusy() && frontright.isBusy()) {
                    //Do Nothing
                }


                backleft.setPower(0);
                backright.setPower(0);
                frontleft.setPower(0);
                frontright.setPower(0);


                backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            }

            if (gamepad1.x) {
                //In degrees

                double initialTime = System.currentTimeMillis();

                double initialZ = getRelativeAngle();
                double finalZ = initialZ - angleChange;

                long lastSampleTime = System.currentTimeMillis();
                double errorSum = 0.0;
                double previousError = 0.0;

                double currentZ = initialZ;
                while (Math.abs(normalizeAngle(currentZ - finalZ)) > threshold && (System.currentTimeMillis() - initialTime) < timeOutMillis) {
                    if (gamepad1.y) {
                        break;
                    }
                    currentZ = getRelativeAngle();

                    double error = normalizeAngle(currentZ - finalZ);

                    //Time Delta in Seconds
                    double timeDelta = (System.currentTimeMillis() - lastSampleTime) / 1000.0;
                    //Error Derivative in degrees/sec
                    double errorDerivative = (error - previousError) / timeDelta;
                    //Antiwind up
                    if (Math.abs(error) < windupThreshold) {
                        errorSum += error * (timeDelta);
                    }
                    lastSampleTime = System.currentTimeMillis();
                    previousError = error;

                    double power = error * pValue + errorDerivative * dValue + errorSum * iValue;
//                    if (power < 0 && Math.abs(power) < 0.1) {
//                        power = -0.1;
//                    }
//                    if (power > 0 && Math.abs(power) < 0.1) {
//                        power = 0.1;
//                    }

                    backleft.setPower(power * rightRotateVector[0]);
                    backright.setPower(power * rightRotateVector[1]);
                    frontleft.setPower(power * rightRotateVector[2]);
                    frontright.setPower(power * rightRotateVector[3]);

                    telemetry.addData("Current Angle: ", getRelativeAngle());
                    telemetry.addData("Target Angle: ", finalZ);
                    telemetry.addData("Error: ", finalZ);
                    telemetry.addData("Error Derivative: ", finalZ);
                    telemetry.addData("Error Sum: ", finalZ);
                    telemetry.update();
                }

                backleft.setPower(0);
                backright.setPower(0);
                frontleft.setPower(0);
                frontright.setPower(0);

            }

            telemetry.update();
        }
    }


    //Normalize angle to be in -180 to 180
    public double normalizeAngle(double angle) {
        double sign = (angle < 0) ? -1.0 : 1.0;
        double magnitude = Math.abs(angle) % 360;
        if (magnitude > 180) {
            return sign * (magnitude - 360);
        } else {
            return sign * magnitude;
        }
    }
    public double getRelativeAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
}