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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.TestBot;


@Autonomous(name="CarrickMotorEncoderTest_Auto", group="rex")
//@Disabled
public class CarrickMotorEncoderTest_Auto extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private TestBot robot = null;

    @Override
    public void runOpMode() {
        robot = new TestBot(this);
        robot.navigator.init();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        double power = .25;

        robot.driveTrain.moveForward(.5, power);
        robot.navigator.rotate(-90, power);
        robot.driveTrain.moveForward(.5, power);
        robot.navigator.rotate(-90, power);
        robot.driveTrain.moveForward(.5, power);
        robot.navigator.rotate(-90, power);
        robot.driveTrain.moveForward(.5, power);
        robot.navigator.rotate(-90, power);

        robot.driveTrain.crabLeft(1);
        robot.driveTrain.crabRight(1);
/*
        // move forward for a number of seconds at specific power
        robot.driveTrain.moveForward(.5, power);
        robot.driveTrain.turnLeft(.5, power);
        robot.driveTrain.turnRight(.5, power);
        robot.driveTrain.moveBackward(.5, power);
*/
        robot.driveTrain.resetEncoders();
        robot.driveTrain.encoderDrive(.25, 5, 5, 5);
        robot.driveTrain.encoderDrive(.25, -5, 5, 5);
        robot.driveTrain.encoderDrive(.25, 5, 5, 5);
        robot.driveTrain.encoderDrive(.25, -5, 5, 5);
        robot.driveTrain.encoderDrive(.25, 5, 5, 5);
        robot.driveTrain.encoderDrive(.25, -5, 5, 5);
        robot.driveTrain.encoderDrive(.25, 5, 5, 5);
        robot.driveTrain.encoderDrive(.25, -5, 5, 5);

        // Show the elapsed game time.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

    }
}