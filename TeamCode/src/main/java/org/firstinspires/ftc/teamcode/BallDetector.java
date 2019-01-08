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

import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.Environment;
import android.view.ViewGroup;
import android.widget.ImageView;
import android.widget.RelativeLayout;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.CameraDevice;
import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
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

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
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

@TeleOp(name="CV Color Detector", group ="Concept")
public class BallDetector extends LinearOpMode {

    public static final String TAG = "Vuforia Navigation Sample";
    OpenGLMatrix lastLocation = null;

    VuforiaLocalizer vuforia;


    @Override public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AQbo49L/////AAABmevTy7Z03UsqrGI3R60ynpp4g2rvNVf/1dCnq/yA0t/udppqkvr8wuV7EJUTuEa5rpWh172gpT25p58RhAeE2g5ulDjPlko+sREUhxbr5Nlu7T0dcljWhUCWoTe2wVPN+pI4TRbfsXfTKLhPxDsk8H4uXFFXLoMNrLV/cv83mGrUpSGmYDqVQczcgKcZjKCDbxC0QtOv5alUZfT9Qt6rrIYFG0ZCoVUnq64B6yeDS4P49yo5czr1LQ/9ZOjGeoaFzOG18WjxKoNnr7dffjEwvEj9p/uaUYT025Bzu7w6J0SqRcc0+BGAkPVTr54/sxJIL82+UEWzkApu5g6xDrYT5+s/dDh+aS59TYElhpGOuFeI";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.fillCameraMonitorViewParent = false;

        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        vuforia.setFrameQueueCapacity(1);
        vuforia.enableConvertFrameToBitmap();

        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

//        CameraDevice.getInstance().setFlashTorchMode(true);
        double threshold = .95;
        int minVotes = 30;
        int minRadius = 1;
        int maxRadius = 10;

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                minVotes++;
                sleep(100);
            }
            if (gamepad1.dpad_down) {
                minVotes--;
                sleep(100);
            }
            if (gamepad1.dpad_right) {
                maxRadius++;
                sleep(100);
            }
            if (gamepad1.dpad_left) {
                maxRadius--;
                sleep(100);
            }
            if (gamepad1.x) {
                threshold += .01;
                sleep(100);
            }
            if (gamepad1.b) {
                threshold -= 0.01;
                sleep(100);
            }
            if (gamepad1.left_bumper) {
                minRadius -= 1;
                sleep(100);
            }
            if (gamepad1.right_bumper) {
                minRadius += 1;
                sleep(100);
            }
            telemetry.addData("Min Votes: ", minVotes);
            telemetry.addData("Min Radius: ", minRadius);
            telemetry.addData("Max Radius: ", maxRadius);
            telemetry.addData("Threshold ", threshold);
            telemetry.update();
            if (gamepad1.a) {
                try {
                    Frame frame = vuforia.getFrameQueue().take();
                    for (int i = 0; i < frame.getNumImages(); i++) {
                        //Get Image
                        Image image = frame.getImage(i);
                        if (image.getFormat() == PIXEL_FORMAT.RGB565) {
                            BlobDetector detector = new BlobDetector(image, threshold, 0.1);
//                        ArrayList<int[]> blobs = detector.findCircles(30, 10);
//                        Collections.sort(blobs, new Comparator<int[]>() {
//                            @Override
//                            public int compare(int[] t1, int[] t2) {
//                                if (t1[0] < t2[0]) {
//                                    return -1;
//                                }
//                                if (t1[0] > t2[0]) {
//                                    return 1;
//                                }
//                                return 0;
//                            }
//                        });
//                        for (int[] blob : blobs) {
//                            int color = detector.scaledBitmap.getPixel(blob[0], blob[1]);
//                            final float[] hslValue = {0F, 0F, 0F};
//                            Color.colorToHSV(color, hslValue);
//                            telemetry.addData("COLOR H: ", hslValue[0]);
//                            telemetry.addData("COLOR S: ", hslValue[1]);
//                            telemetry.addData("COLOR V: ", hslValue[2]);
//                            telemetry.addData("--------", "---------");
//
//                        }

                            detector.renderImage("/sdcard/DCIM/Camera/preview.png", minVotes, minRadius, maxRadius);
                            telemetry.addData("PHOTO TAKEN ", "");
                            sleep(1000);
//                        telemetry.addData("Blobs: ", blobs.size());
                            telemetry.update();
                            break;
                        }
                    }
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }

}