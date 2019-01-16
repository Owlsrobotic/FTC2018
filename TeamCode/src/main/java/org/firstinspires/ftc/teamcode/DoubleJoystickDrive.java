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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.sql.Driver;


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

@TeleOp(name="Double Joystick Driver OP", group="Linear Opmode")
public class DoubleJoystickDrive extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    RobotController controller;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        controller = new RobotController(this);
        DriverConfiguration cfg = new DriverConfiguration(controller.frontright,
                controller.frontleft,
                controller.backright,
                controller.backleft,
                new int[]{-1, 1, -1, 1},
                0.5);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        double power = 0.6;
        while (opModeIsActive()) {
            if(gamepad1.right_trigger > 0.5) {
                power = Math.min(power + 0.1, 1.0);
                controller.sleep(100);
            }

            if(gamepad1.left_trigger > 0.5) {
                power = Math.max(0.0, power - 0.1);
                controller.sleep(100);
            }

            telemetry.addData("Drive Power: ", power * 100.0 + "%");
            telemetry.update();

            cfg.setMaxPower(power);
            cfg.zeroPower();

            // Collect input
            double transPow = RobotUtil.round(gamepad1.left_stick_y, 0.1);
            double rotPow = RobotUtil.round(gamepad1.right_stick_x, 0.1);

            cfg.addTransPower(transPow);
            cfg.addRotPower(rotPow);
            cfg.startMotors();

            //Reset Motors
            controller.hook.setPower(0.0);
            controller.leftClaw.setPower(0.0);
            controller.rightClaw.setPower(0.0);

            //Arm
            if(gamepad2.left_bumper && !gamepad2.right_bumper) {
                //Lower Arm
                controller.leftClaw.setPower(-0.7);
                controller.rightClaw.setPower(-0.7);
            } else if (gamepad2.right_bumper && !gamepad2.left_bumper) {
                //Raise Arm
                controller.leftClaw.setPower(0.7);
                controller.rightClaw.setPower(0.7);
            }

            //Claw
            if(gamepad2.b && !gamepad2.a) {
                //Open Claw
                controller.claw.setDirection(CRServo.Direction.FORWARD);
                controller.claw.setPower(1.0);
            } else if (gamepad2.a && !gamepad2.b) {
                //Close Claw
                controller.claw.setDirection(CRServo.Direction.REVERSE);
                controller.claw.setPower(1.0);
            } else {
                controller.claw.setPower(0.0);
            }

            //Hook
            if (gamepad1.right_bumper) {
                //Raise Hook
                controller.hook.setPower(-1.0);
            }
            if (gamepad1.left_bumper) {
                //Lower Hook
                controller.hook.setPower(1.0);
            }
        }
    }
}
