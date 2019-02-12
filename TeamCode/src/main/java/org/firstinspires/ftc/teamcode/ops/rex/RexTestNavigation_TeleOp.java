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

package org.firstinspires.ftc.teamcode.ops.rex;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.GameBot;


@TeleOp(name="RexTestNavigation_TeleOp", group="rex")
//@Disabled
public class RexTestNavigation_TeleOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private GameBot robot = null;
    private boolean logEnableTrace = true;

    @Override
    public void runOpMode() {

        robot = new GameBot(this);
        robot.logger.open(logEnableTrace);
        robot.logger.enableTelemetry();
        robot.gyroNavigator.init();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (robot.gyroNavigator.isAvailable) {
            robot.logger.logDebug("runOpMode", "gyroNavigator.isAvailable:%b", robot.gyroNavigator.isAvailable);
            robot.gyroNavigator.resetAngle();
        }

        while (opModeIsActive()) {

            if (robot.gyroNavigator.isAvailable) {
                robot.logger.logDebug("runOpMode", "gyroNavigator.getAngle:%f", robot.gyroNavigator.getAngle());
            }

            // driveTrain controls
            if (robot.driveTrain.isAvailable) {

                double leftX = gamepad1.left_stick_x;
                double leftY = gamepad1.left_stick_y;
                double rightX = gamepad1.right_stick_x;
                double rightY = gamepad1.right_stick_y;

                robot.driveTrain.updateMotorsMechanumDrive(leftX, leftY, rightX, rightY);

                robot.logger.logDebug("runOpMode", "Run Time: %s", runtime.toString());
                robot.logger.logDebug("runOpMode", "Left:  X (%.2f), Y (%.2f)", leftX, leftY);
                robot.logger.logDebug("runOpMode", "Right: X (%.2f), Y (%.2f)", rightX, rightY);

            }

            if (robot.gyroNavigator.isAvailable && robot.driveTrain.isAvailable) {
                if (gamepad1.x){
                    robot.driveTrain.gyroRotate(-90, 0.5);
                }
                if (gamepad1.b){
                    robot.driveTrain.gyroRotate(90, 0.5);
                }
            }

            //encoderDrive2(double Lspeed, double Rspeed, double Inches, double timeoutS, double rampup)
            if (robot.driveTrain.isAvailable) {
                if (gamepad1.dpad_up) {
                    robot.driveTrain.resetEncoders();
                    robot.driveTrain.encoderDrive(.5, 12, 12, 5);
                    //robot.driveTrain.encoderDrive2(.25, .25, 5, 10, 0);
                }
            }

            telemetry.update();
        }

        // Show the elapsed game time.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

    }
}
