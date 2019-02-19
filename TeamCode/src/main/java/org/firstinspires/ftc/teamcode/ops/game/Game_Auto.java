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

package org.firstinspires.ftc.teamcode.ops.game;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.GameBot;
import org.firstinspires.ftc.teamcode.components.DriveTrain;
import org.firstinspires.ftc.teamcode.components.WebCamera;


@Autonomous(name="Game_Auto", group="game")
//@Disabled
public class Game_Auto extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private GameBot robot = null;
    private boolean logEnableTrace = true;
    private boolean logToTelemetry = true;


    @Override
    public void runOpMode() {

        robot = new GameBot(this, logEnableTrace, logToTelemetry);
        robot.logger.logInfo("runOpMode", "===== [ Start Initializing ]");

        /* Use either robot.initAll or select only the components that need initializing below */
        //robot.initAll();
        robot.gyroNavigator.init();
        robot.driveTrain.init(DriveTrain.InitType.INIT_4WD);
        robot.webCamera.init(WebCamera.InitType.INIT_FOR_FIND_GOLD);
        robot.goldSensor.init();

        robot.logger.logInfo("runOpMode", "===== [ Initialization Complete ]");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.logger.logInfo("runOpMode", "===== [ Start Autonomous ]");
        runtime.reset();


        robot.logger.logInfo("runOpMode", "===== [ Lower Robot ]");
        robot.hoist.contractedPosition = 0;
        robot.hoist.extendedPosition = 23000;
        robot.hoist.rampUpDownThreshold = 1;
        robot.hoist.power = 1;

        robot.hoist.extend();

        robot.logger.logInfo("runOpMode", "===== [ Look for Gold ]");
        String goldPosition = robot.goldSensor.goldFind(5);

        robot.logger.logInfo("runOpMode", "===== [ Move Off Lander ]");
        robot.driveTrain.crabRight(0.4);

        robot.logger.logInfo("runOpMode", "===== [ Look for Gold ]");
        goldPosition = robot.goldSensor.goldFind(5);

        robot.logger.logInfo("runOpMode", "===== [ Adjust Angle ]");
        robot.driveTrain.gyroRotate(0.5, 0, false);


        robot.logger.logInfo("runOpMode", "===== [ Move Toward Gold ]");
        robot.logger.logInfo("runOpMode", "goldPosition: %s", goldPosition);

        double goldPositionOffset = 0;

        switch (goldPosition) {
            case "Right":

                robot.logger.logInfo("runOpMode", "===== [ Gold on Right ]");
                robot.driveTrain.encoderDrive(1, -13);
                robot.driveTrain.crabLeft(1.7);
                robot.driveTrain.encoderDrive(1, -15);
                goldPositionOffset = 32;
                break;

            case "Center":

                robot.logger.logInfo("runOpMode", "===== [ Gold in Center ]");
                robot.driveTrain.encoderDrive(1,-20);
                robot.driveTrain.crabLeft(0.4);
                robot.driveTrain.encoderDrive(1, -8);
                goldPositionOffset = 16;
                break;

            case "Left":

                robot.logger.logInfo("runOpMode", "===== [ Gold on Left ]");
                robot.driveTrain.encoderDrive(1, -13);
                robot.driveTrain.crabRight(0.75);
                robot.driveTrain.encoderDrive(1, -15);
                goldPositionOffset = 0;
                break;

            default:
                robot.logger.logInfo("runOpMode", "===== [ Gold ??? ]");
                //                telemetry.addData("Gold:", "???");

        }

        robot.logger.logInfo("runOpMode", "===== [ Back up and head for Depot ]");
        robot.driveTrain.encoderDrive(1, 9);
        robot.driveTrain.gyroRotate(-90, 0.75, false);
        robot.driveTrain.encoderDrive(1, -28.0 - goldPositionOffset);
        robot.driveTrain.gyroRotate(-44, 0.75);
        robot.driveTrain.encoderDrive(1, -36);
        robot.driveTrain.encoderDrive(1, 63);


        // Show the elapsed game time.
        robot.logger.logInfo("runOpMode", "===== [ Autonomous Complete ] Run Time: %s", runtime.toString());
        telemetry.update();

    }
}
