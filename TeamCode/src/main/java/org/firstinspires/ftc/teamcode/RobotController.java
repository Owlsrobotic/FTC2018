package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
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
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**
 * RobotController controls all hardware on the robot
 * and abstracts all computation done by the robot
 */

public class RobotController {
    //Localization Data in meters and degrees
    public double x = 0.0;
    public double y = 0.0;
    public double angle = 0.0;

    // color sensor
    public ColorSensor colorSensor;
    public int[] WHITE_RGB = {255, 255, 255};
    public int[] GOLD_RGB = {255, 215, 0};

    private LinearOpMode context;
    private HardwareMap hmap;

    //Movement Stuff
    double[] fowardVector = {-1.0,1.0,-1.0,1.0};
    double[] rightRotateVector = {-1.0, -1.0, -1.0, -1.0};

    private DcMotor backleft = null;
    private DcMotor backright = null;
    private DcMotor frontleft = null;
    private DcMotor frontright = null;

    //Movement Calibration Stuff
    double     COUNTS_PER_MOTOR_REV    = 2240 ;
    //Good Number: 0.51625
    double     TRANSLATION_FACTOR    = 0.5 ;
    double     WHEEL_DIAMETER_M   = 0.1016;
    double     COUNTS_PER_M         = (COUNTS_PER_MOTOR_REV * TRANSLATION_FACTOR) /
            (WHEEL_DIAMETER_M * 3.1415);

    double timeOutMillis = 3000;
    double threshold = 0.05;
    //Good Number: 0.04456049
    double pValue = 0.04456049;

    //Vuforia Stuff
    OpenGLMatrix lastLocation = null;

    VuforiaLocalizer vuforia;

    double FIELD_LENGTH_M = 3.6576;
    double WALL_DISTANCE_FROM_ORIGIN_M = FIELD_LENGTH_M / 2.0;

    double phoneXDisplacementCenterM = 0.0;
    double phoneYDisplacementCenterM = 0.208;

    HashMap<String, PosRot> markerPosition;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    String bluePerimeterKey = "BluePerimeter";
    String redPerimeterKey = "RedPerimeter";
    String frontPerimeterKey = "FrontPerimeter";
    String backPerimeterKey = "BackPerimeter";

    //IMU Stuff
    BNO055IMU imu;
    BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();

    public RobotController(LinearOpMode context) {
        this.context = context;

        hmap = context.hardwareMap;

        // setting up motors
        frontright = hmap.dcMotor.get("front_right");
        frontleft = hmap.dcMotor.get("front_left");
        backright = hmap.dcMotor.get("back_right");
        backleft = hmap.dcMotor.get("back_left");

        // sensors

        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Initialize Vuforia stuff
        markerPosition = new HashMap<>();
        markerPosition.put(frontPerimeterKey, new PosRot(0.0, WALL_DISTANCE_FROM_ORIGIN_M, 0));
        markerPosition.put(backPerimeterKey, new PosRot(0.0, -1.0 * WALL_DISTANCE_FROM_ORIGIN_M, 180));
        markerPosition.put(redPerimeterKey, new PosRot(-1.0 * WALL_DISTANCE_FROM_ORIGIN_M, 0.0, -90));
        markerPosition.put(bluePerimeterKey, new PosRot(WALL_DISTANCE_FROM_ORIGIN_M, 0.0, 90));

        int cameraMonitorViewId = hmap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hmap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AQbo49L/////AAABmevTy7Z03UsqrGI3R60ynpp4g2rvNVf/1dCnq/yA0t/udppqkvr8wuV7EJUTuEa5rpWh172gpT25p58RhAeE2g5ulDjPlko+sREUhxbr5Nlu7T0dcljWhUCWoTe2wVPN+pI4TRbfsXfTKLhPxDsk8H4uXFFXLoMNrLV/cv83mGrUpSGmYDqVQczcgKcZjKCDbxC0QtOv5alUZfT9Qt6rrIYFG0ZCoVUnq64B6yeDS4P49yo5czr1LQ/9ZOjGeoaFzOG18WjxKoNnr7dffjEwvEj9p/uaUYT025Bzu7w6J0SqRcc0+BGAkPVTr54/sxJIL82+UEWzkApu5g6xDrYT5+s/dDh+aS59TYElhpGOuFeI";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables roverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable bluePerimeter = roverRuckus.get(0);
        bluePerimeter.setName(bluePerimeterKey);

        VuforiaTrackable redPerimeter  = roverRuckus.get(1);
        redPerimeter.setName(redPerimeterKey);

        VuforiaTrackable frontPerimeter  = roverRuckus.get(2);
        frontPerimeter.setName(frontPerimeterKey);

        VuforiaTrackable backPerimeter  = roverRuckus.get(3);
        backPerimeter.setName(backPerimeterKey);

        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables.addAll(roverRuckus);
        roverRuckus.activate();

        //Initialize IMU Stuff
        imuParameters.loggingEnabled = true;
        imuParameters.loggingTag     = "IMU";
        imuParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.calibrationDataFile = "IMUCalibration.json";
        imuParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hmap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParameters);
    }

    public void waitForUserInput() {
        context.telemetry.addData("WAITING FOR INPUT:", "PRESS A on Gamepad 1 TO PROCEED");
        context.telemetry.update();
        while (true) {
            if (context.gamepad1.a) {
                break;
            }
        }
        context.telemetry.clearAll();
    }

    /**
     * Rotates the robot.
     *
     * @param angleChange angle in degrees
     */
    public void rotateAngle(double angleChange) {
        angle += angleChange;

        //Normalize angle to be in -180 to 180
        double sign = (angleChange < 0) ? -1.0 : 1.0;
        double magnitude = Math.abs(angleChange) % 360;
        if (magnitude > 180) {
            angleChange = sign * (magnitude - 360);
        } else {
            angleChange = sign * magnitude;
        }

        //Reset measurements
        imu.initialize(imuParameters);

        double initialTime = System.currentTimeMillis();

        double initialZ = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double finalZ = initialZ - angleChange;

        double currentZ = initialZ;
        while (Math.abs(currentZ - finalZ) > threshold && (System.currentTimeMillis() - initialTime) < timeOutMillis) {
            currentZ = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            double error = currentZ - finalZ;

            double power = error * pValue;
            if (power < 0 && Math.abs(power) < 0.1) {
                power = -0.1;
            }
            if (power > 0 && Math.abs(power) < 0.1) {
                power = 0.1;
            }

            backleft.setPower(power * rightRotateVector[0]);
            backright.setPower(power * rightRotateVector[1]);
            frontleft.setPower(power * rightRotateVector[2]);
            frontright.setPower(power * rightRotateVector[3]);

        }

        backleft.setPower(0);
        backright.setPower(0);
        frontleft.setPower(0);
        frontright.setPower(0);
    }

    /**
     * Moves the robot.
     *
     * @param distance distance in meters
     */
    public void moveDistanceForward(double distance) {
        double cartesianAngle = PosRot.convertToCartesianAngle(angle);
        x += Math.cos(Math.toRadians(cartesianAngle));
        y += Math.sin(Math.toRadians(cartesianAngle));

        double power = 0.5;
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

    //Move to PosRot location but disregards the rotation data
    public void moveToLocation(PosRot posRot) {
        double finalAngle = Math.toDegrees(Math.atan2(posRot.y - y, posRot.x - x));
        finalAngle = PosRot.convertToFieldAngle(finalAngle);

        double angleChange = finalAngle - angle;

//        context.telemetry.addData("Angle Change: ", angleChange);
//        context.telemetry.addData("Bot X: ", x);
//        context.telemetry.addData("Bot Y: ", y);
//        context.telemetry.addData("Bot Rot: ", angle);
//        context.telemetry.addData("Target X: ", posRot.x);
//        context.telemetry.addData("Target Y: ", posRot.y);

        waitForUserInput();

        //Point robot to the endpoint ... move there ... update robot's internal location
        double distance = Math.sqrt(Math.pow(posRot.y - y, 2) + Math.pow(posRot.x - x, 2));
        rotateAngle(angleChange);

        waitForUserInput();

        moveDistanceForward(distance);

    }

    public PosRot localize() {
        //How far the image is relative to the camera(in millimeters)
        double tX = 0.0;
        double tZ = 0.0;
        //How the image is rotated relative to the camera(in degrees)
        double rY = 0.0;

        String trackerName = "";

        //Look for all tracker markers
        for (VuforiaTrackable trackable : allTrackables) {
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) trackable.getListener()).getPose();
            if (pose != null) {
                VectorF trans = pose.getTranslation();
                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                tX = trans.get(0);
                tZ = trans.get(2);

                rY = rot.secondAngle;

                trackerName = trackable.getName();
            }
        }
        /**
         * Provide feedback as to where the robot was last located (if we know).
         */
        if (tX != 0.0) {
            PosRot markerRotPos = markerPosition.get(trackerName);

            PosRot robotRotPos = new PosRot(tX / 1000.0, tZ / 1000.0, rY);
            robotRotPos = robotRotPos.rotate(-1.0 * robotRotPos.rot);
            robotRotPos = robotRotPos.rotate(-1.0 * markerRotPos.rot);

            //Global position(meters) and global angle(degrees) of phone
            angle = markerRotPos.rot + robotRotPos.rot;

            //Need to make the position of the robot relative to the center ... NOT the phone camera
            PosRot phoneDisplacement = new PosRot(phoneXDisplacementCenterM, phoneYDisplacementCenterM, 0.0);
            phoneDisplacement = phoneDisplacement.rotate(-1.0 * angle);

            x = markerRotPos.x + robotRotPos.x - phoneDisplacement.x;
            y = markerRotPos.y + robotRotPos.y - phoneDisplacement.y;

            return new PosRot(x, y, angle);

        } else {
            return null;
        }
    }
}