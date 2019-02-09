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

package org.firstinspires.ftc.teamcode.ops.ethan;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.GameBot;


@TeleOp(name="Ethan_Game_TeleOp", group="ethan")
//@Disabled
public class Ethan_Game_TeleOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private GameBot robot = null;
    private boolean logEnableTrace = true;

    @Override
    public void runOpMode() {
        robot = new GameBot(this);
        robot.logger.open(logEnableTrace);

        if(robot.navigator.isAvailable){
            robot.navigator.initLocations();
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        robot.hoist.contractedPosition = 0;
        robot.hoist.extendedPosition = 20000;
        robot.hoist.rampUpDownThreshold = 500;
        robot.hoist.power = 1;

        while (opModeIsActive()) {

            if (robot.navigator.isAvailable){
                robot.navigator.displayLocationInfo();
            }

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

            // goldSensor detection
            if(robot.goldSensor.isAvailable) {
                telemetry.addData("goldDirection:", robot.goldSensor.goldFind());
            }

            telemetry.update();
            //PNP controls
            if (robot.pnp.isAvailable){
                if (gamepad1.right_stick_y > 0){
                    robot.pnp.extend();
                }
                else if(gamepad1.right_stick_y < 0) {
                    robot.pnp.contract();
                }
                else{
                    robot.pnp.pusher.setPower(0.0);
                }
            }
            telemetry.addData("rightsticky", gamepad1.right_stick_y);

            robot.arm.crank.setPower(-gamepad1.left_trigger+gamepad1.right_trigger);
        }

        // Show the elapsed game time.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

    }
}
