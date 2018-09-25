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

import android.hardware.Camera;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**
 * This 2016-2017 OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the FTC field.
 * The code is structured as a LinearOpMode
 *
 * Vuforia uses the phone's camera to inspect it's surroundings, and attempt to locate target images.
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code than combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * This example assumes a "diamond" field configuration where the red and blue alliance stations
 * are adjacent on the corner of the field furthest from the audience.
 * From the Audience perspective, the Red driver station is on the right.
 * The two vision target are located on the two walls closest to the audience, facing in.
 * The Stones are on the RED side of the field, and the Chips are on the Blue side.
 *
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

@Autonomous(name="Concept: Vuforia Navigation", group ="Concept")
public class VuforiaNav extends LinearOpMode {

    public static final String TAG = "Vuforia Navigation Sample";
    OpenGLMatrix lastLocation = null;

    VuforiaLocalizer vuforia;

    double FIELD_LENGTH_M = 3.6576;
    double WALL_DISTANCE_FROM_ORIGIN_M = FIELD_LENGTH_M / 2.0;

    HashMap<String, PosRot> markerPosition;
    String bluePerimeterKey = "BluePerimeter";
    String redPerimeterKey = "RedPerimeter";
    String frontPerimeterKey = "FrontPerimeter";
    String backPerimeterKey = "BackPerimeter";

    @Override public void runOpMode() {
        markerPosition = new HashMap<>();
        markerPosition.put(frontPerimeterKey, new PosRot(0.0, WALL_DISTANCE_FROM_ORIGIN_M, 0));
        markerPosition.put(backPerimeterKey, new PosRot(0.0, -1.0 * WALL_DISTANCE_FROM_ORIGIN_M, 180));
        markerPosition.put(redPerimeterKey, new PosRot(-1.0 * WALL_DISTANCE_FROM_ORIGIN_M, 0.0, -90));
        markerPosition.put(bluePerimeterKey, new PosRot(WALL_DISTANCE_FROM_ORIGIN_M, 0.0, 90));

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
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
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(roverRuckus);


        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        /** Start tracking the data sets we care about. */
        roverRuckus.activate();

        CameraDevice.getInstance().setFlashTorchMode(true);
        while (opModeIsActive()) {

            //How far the image is relative to the camera(in millimeters)
            double tX = 0.0;
            double tY = 0.0;
            double tZ = 0.0;
            //How the image is rotated relative to the camera(in degrees)
            double rX = 0.0;
            double rY = 0.0;
            double rZ = 0.0;

            String trackerName = "";

            for (VuforiaTrackable trackable : allTrackables) {
                /**
                 * getUpdatedRobotLocation() will return null if no new information is available since
                 * the last time that call was made, or if the trackable is not currently visible.
                 * getRobotLocation() will return null if the trackable is not currently visible.
                 */
                telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) trackable.getListener()).getPose();
                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                    tX = trans.get(0);
                    tY = trans.get(1);
                    tZ = trans.get(2);

                    rX = rot.firstAngle;
                    rY = rot.secondAngle;
                    rZ = rot.thirdAngle;

                    trackerName = trackable.getName();
                }
            }
            /**
             * Provide feedback as to where the robot was last located (if we know).
             */
            if (tX != 0.0) {
                telemetry.addData("Trans X: ", tX);
                telemetry.addData("Trans Y: ", tY);
                telemetry.addData("Trans Z: ", tZ);

                telemetry.addData("Rot X: ", rX);
                telemetry.addData("Rot Y: ", rY);
                telemetry.addData("Rot Z: ", rZ);

                PosRot markerRotPos = markerPosition.get(trackerName);

                PosRot robotRotPos = new PosRot(tX / 1000.0, tZ / 1000.0, rY);
                robotRotPos = robotRotPos.rotate(-1.0 * markerRotPos.rot);

                telemetry.addData("Robot Pos X: ", markerRotPos.x + robotRotPos.x);
                telemetry.addData("Robot Pos Y: ", markerRotPos.y + robotRotPos.y);
                telemetry.addData("Robot Rot: ", markerRotPos.rot + robotRotPos.rot);

            } else {
                telemetry.addData("Pos", "Unknown");
            }
            telemetry.update();
        }
    }

}
