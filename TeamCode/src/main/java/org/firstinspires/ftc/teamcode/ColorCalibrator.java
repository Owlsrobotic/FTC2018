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
import android.content.Context;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;


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

@TeleOp(name="Color Calibrator", group="Linear Opmode")
public class ColorCalibrator extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    View relativeLayout;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        final ColorSensor color = hardwareMap.colorSensor.get("color");

        ArrayList<ColorDataTrain> trainingSet = new ArrayList<>();
        boolean hasLoaded = false;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            final float[] hslValue = {0F, 0F, 0F};
            Color.RGBToHSV((int) (color.red() * 255),
                    (int) (color.green() * 255),
                    (int) (color.blue() * 255),
                    hslValue);

            telemetry.addData("H", hslValue[0]);
            telemetry.addData("S", hslValue[1]);
            telemetry.addData("L", hslValue[2]);
            telemetry.addData("Data Size", trainingSet.size());
            telemetry.addData("Controls: ", "PRESS X, Y, B, START, DPAD_DOWN for NONE, GOLD, WHITE, SAVE, LOAD");

            final float[] hslCopy = {hslValue[0], hslValue[1], hslValue[2]};
            if (gamepad1.x) {
                trainingSet.add(new ColorDataTrain(hslCopy, ColorDataTrain.COLOR_NONE));
            }
            if (gamepad1.y) {
                trainingSet.add(new ColorDataTrain(hslCopy, ColorDataTrain.COLOR_GOLD));
            }
            if (gamepad1.b) {
                trainingSet.add(new ColorDataTrain(hslCopy, ColorDataTrain.COLOR_WHITE));
            }
            if (gamepad1.start) {
                try {
                    FileOutputStream fos = null;
                    fos = hardwareMap.appContext.openFileOutput("ColorCalibration.ser", Context.MODE_PRIVATE);
                    ObjectOutputStream os = new ObjectOutputStream(fos);
                    os.writeObject(trainingSet);
                    os.close();
                    fos.close();
                } catch (Exception e) {
                    e.printStackTrace();
                }
                break;
            }
            if (gamepad1.dpad_down) {
                hasLoaded = true;
                try {
                    FileInputStream fis = hardwareMap.appContext.openFileInput("ColorCalibration.ser");
                    ObjectInputStream is = new ObjectInputStream(fis);
                    trainingSet = (ArrayList<ColorDataTrain>) is.readObject();
                    is.close();
                    fis.close();
                } catch (Exception e){}
            }
            if (hasLoaded && gamepad1.dpad_up) {
                int k = 3;
                Collections.sort(trainingSet, new Comparator<ColorDataTrain>() {
                    @Override
                    public int compare(ColorDataTrain e1, ColorDataTrain e2) {
                        float[] hsl1 = e1.hsl;
                        float[] hsl2 = e2.hsl;

                        double distance1 = Math.sqrt(Math.pow(hsl1[0] - hslCopy[0], 2) + Math.pow(hsl1[1] - hslCopy[1], 2)+ Math.pow(hsl1[2] - hslCopy[2], 2));
                        double distance2 = Math.sqrt(Math.pow(hsl2[0] - hslCopy[0], 2) + Math.pow(hsl2[1] - hslCopy[1], 2)+ Math.pow(hsl2[2] - hslCopy[2], 2));
                        return Double.compare(distance1, distance2);
                    }
                });

                int goldHits = 0;
                int whiteHits = 0;
                int emptyHits = 0;
                for (int i = 0; i < k; i++) {
                    if (trainingSet.get(i).color == ColorDataTrain.COLOR_NONE) {
                        emptyHits++;
                    } else if (trainingSet.get(i).color == ColorDataTrain.COLOR_GOLD) {
                        goldHits++;
                    } else if (trainingSet.get(i).color == ColorDataTrain.COLOR_WHITE) {
                        whiteHits++;
                    }
                    telemetry.addData("Color Data " + i, trainingSet.get(i).color);
                }
                //Start with classification as empty then filter down
                int classification = ColorDataTrain.COLOR_NONE;
                if (goldHits > emptyHits && goldHits > whiteHits) {
                    classification = ColorDataTrain.COLOR_GOLD;
                }
                if (whiteHits > emptyHits && whiteHits > goldHits) {
                    classification = ColorDataTrain.COLOR_WHITE;
                }

                telemetry.addData("Classification (0: NONE, 1: WHITE, 2: GOLD): ", classification);
                telemetry.update();

                try {
                    Thread.sleep(3000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            telemetry.update();


//            relativeLayout.post(new Runnable() {
//                public void run() {
//                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, hslValue));
//                }
//            });
        }
    }
}
