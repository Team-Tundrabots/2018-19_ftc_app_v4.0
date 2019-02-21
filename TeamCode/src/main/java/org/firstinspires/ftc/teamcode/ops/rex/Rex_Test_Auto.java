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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.GameBot;
import org.firstinspires.ftc.teamcode.components.DriveTrain;
import org.firstinspires.ftc.teamcode.components.WebCamera;


@Autonomous(name="Rex_Test_Auto", group="rex")
@Disabled
public class Rex_Test_Auto extends LinearOpMode {

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

        robot.logger.logInfo("runOpMode", "===== [ Initialization Complete ]");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.logger.logInfo("runOpMode", "===== [ Start Autonomous ]");
        runtime.reset();

        robot.logger.logInfo("runOpMode", "========================================================");
        robot.logger.logInfo("runOpMode", "===== [ crabEncoderLeft ]");
        robot.logger.logInfo("runOpMode", "========================================================");
        robot.driveTrain.crabEncoderLeft(1, 12);

        robot.logger.logInfo("runOpMode", "========================================================");
        robot.logger.logInfo("runOpMode", "===== [ crabEncoderRight ]");
        robot.logger.logInfo("runOpMode", "========================================================");
        robot.driveTrain.crabEncoderRight(1, 12);
/*
        robot.logger.logInfo("runOpMode", "========================================================");
        robot.logger.logInfo("runOpMode", "===== [ Test Gyro: absolute(false), adjust(false) ]");
        robot.logger.logInfo("runOpMode", "========================================================");
        robot.driveTrain.gyroRotate(180, 1, false, false);
        robot.driveTrain.pause(1);
*/

/*
        robot.logger.logInfo("runOpMode", "========================================================");
        robot.logger.logInfo("runOpMode", "===== [ Test Gyro: absolute(false), adjust(true) ]");
        robot.logger.logInfo("runOpMode", "========================================================");
        robot.driveTrain.gyroRotate(90, 0.5, false, true);
        robot.driveTrain.pause(1);
        robot.logger.logInfo("runOpMode", "========================================================");
        robot.logger.logInfo("runOpMode", "===== [ Test Gyro: relative(true), adjust(false) ]");
        robot.logger.logInfo("runOpMode", "========================================================");
        robot.driveTrain.gyroRotate(90, 0.5, true, false);
        robot.driveTrain.pause(1);
        robot.logger.logInfo("runOpMode", "========================================================");
        robot.logger.logInfo("runOpMode", "===== [ Test Gyro: relative(true), adjust(true) ]");
        robot.logger.logInfo("runOpMode", "========================================================");
        robot.driveTrain.gyroRotate(90, 0.5, true, true);
*/
        // Show the elapsed game time.
        robot.logger.logInfo("runOpMode", "===== [ Autonomous Complete ] Run Time: %s", runtime.toString());
        telemetry.update();

    }
}
