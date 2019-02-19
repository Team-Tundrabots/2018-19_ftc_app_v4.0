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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.GameBot;


@Autonomous(name="Game_Auto_Carrick", group="game")
@Disabled
public class Game_Auto_Carrick extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private GameBot robot = null;
    private boolean logEnableTrace = true;

    @Override
    public void runOpMode() {
        robot = new GameBot(this);
        robot.logger.open(logEnableTrace);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        robot.hoist.contractedPosition = 0;
        robot.hoist.extendedPosition = 24000;
        robot.hoist.rampUpDownThreshold = 1;
        robot.hoist.power = 1;


        robot.hoist.extend();

        robot.driveTrain.crabRight(0.35);

        String goldPosition = robot.goldSensor.goldFind();
        while(opModeIsActive() && goldPosition == "Unknown") {
            goldPosition = robot.goldSensor.goldFind();
        }

        telemetry.addData("goldDirection:", goldPosition);
        switch (goldPosition) {
            case "Right":
                robot.driveTrain.encoderDrive(0.25, 0.15, 0.15, 2);
                robot.driveTrain.crabLeft(1.5);
                //robot.driveTrain.moveForward(1,0.25);

            case "Center":
                /*robot.driveTrain.encoderDrive(0.25, 2, 2, 2); */
                //robot.driveTrain.moveForward(0.75,0.25);
                robot.driveTrain.crabLeft(0.4);
                //robot.driveTrain.moveForward(2,0.25);
                stop();

            case "Left":
                robot.driveTrain.encoderDrive(0.25, 0.12, 0.12, 2);
                robot.driveTrain.crabRight(1);
                //robot.driveTrain.moveForward(1,0.25);

            default:
//                telemetry.addData("Gold:", "???");

        }

        // Show the elapsed game time.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

    }
}
