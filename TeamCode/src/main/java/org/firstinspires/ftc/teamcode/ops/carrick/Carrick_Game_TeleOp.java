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

package org.firstinspires.ftc.teamcode.ops.carrick;

import com.qualcomm.ftccommon.SoundPlayer;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.*;
import org.firstinspires.ftc.teamcode.components.DriveTrain;
import org.firstinspires.ftc.teamcode.components.WebCamera;

import java.io.File;


@TeleOp(name="Carrick_Game_TeleOp", group="carrick")
@Disabled
public class Carrick_Game_TeleOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private TestBot robot = null;
    private boolean logEnableTrace = true;
    private boolean logToTelemetry = true;

    @Override
    public void runOpMode() {
        robot = new TestBot(this, logEnableTrace, logToTelemetry);
        robot.logger.logInfo("runOpMode", "===== [ Start Initializing ]");

        //robot.gyroNavigator.init();
        //robot.driveTrain.init(DriveTrain.InitType.INIT_4WD);
        //robot.webCamera.init(WebCamera.InitType.INIT_FOR_FIND_GOLD);
        //robot.goldSensor.init();

        robot.logger.open(logEnableTrace);
        robot.logger.logDebug("TeleOP debug", "");
        robot.logger.logInfo("TeleOP info", "");
        //logger.logDebug("PNP.construct", "");

        int silverSoundID = hardwareMap.appContext.getResources().getIdentifier("silver", "raw", hardwareMap.appContext.getPackageName());
        int goldSoundID   = hardwareMap.appContext.getResources().getIdentifier("gold",   "raw", hardwareMap.appContext.getPackageName());

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        robot.hoist.contractedPosition = 0;
        robot.hoist.extendedPosition = 20000;
        robot.hoist.rampUpDownThreshold = 500;
        robot.hoist.power = .50;
        telemetry.addData("firstrightsticky", gamepad1.right_stick_y);

        boolean goldFound;
        boolean silverFound;
        int soundCounterGold = 0;
        int soundCounterSilver = 0;

        if (goldSoundID != 0)
            goldFound   = SoundPlayer.getInstance().preload(hardwareMap.appContext, goldSoundID);

        if (silverSoundID != 0)
            silverFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, silverSoundID);

        while (opModeIsActive()) {

            // hoist controls
            if (robot.hoist.isAvailable) {
                if (gamepad1.dpad_down) {
                    robot.logger.logDebug("runOpMode", "dpad_down");
                    robot.hoist.extend();
                }

                if (gamepad1.dpad_up) {
                    robot.logger.logDebug("runOpMode", "dpad_up");
                    robot.hoist.contract();
                }
            }

            // driveTrain controls
            if (robot.driveTrain.isAvailable) {
                double leftX = gamepad1.left_stick_x;
                double leftY = gamepad1.left_stick_y;
                double rightX = gamepad1.right_stick_x;
                double rightY = gamepad1.right_stick_y;

                robot.driveTrain.updateMotorsMechanumDrive(leftX, leftY, rightX, rightY);

                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Left", "X (%.2f), Y (%.2f)", leftX, leftY);
                telemetry.addData("Right", "X (%.2f), Y (%.2f)", rightX, rightY);
            }

            //PNP controls
            if (robot.pnp.isAvailable){
                if (gamepad1.right_stick_y > 0){
                    robot.pnp.extend();
                    if(soundCounterGold == 0){
                        SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, goldSoundID);
                        telemetry.addData("Playing", "Resource Gold");
                        telemetry.update();
                    }
                }
                else if(gamepad1.right_stick_y < 0) {
                    robot.pnp.contract();
                    if(soundCounterSilver == 0){
                        SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, silverSoundID);
                        telemetry.addData("Playing", "Resource Silver");
                        telemetry.update();
                    }

                }
                else{
                    robot.pnp.pusher.setPower(0.0);
                }
                double arm_proportion = 0.5;
                double new_left_trigger = gamepad1.left_trigger*arm_proportion;
                double new_right_trigger = gamepad1.right_trigger*arm_proportion;
                robot.arm.crank.setPower(-new_left_trigger+new_right_trigger);
            }
            telemetry.addData("rightsticky", gamepad1.right_stick_y);

            // goldSensor detection
            if(robot.goldSensor.isAvailable) {
                telemetry.addData("goldDirection:", robot.goldSensor.goldFind());
            }

            telemetry.update();
        }

        // Show the elapsed game time.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

    }
}
