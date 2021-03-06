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

package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

// test - adding a comment
public class Arm extends BotComponent {

    public DcMotor crank = null;
    public TouchSensor forwardGuardSwitch = null;
    public TouchSensor backwardGuardSwitch = null;

    /* Constructor */
    public Arm() {

    }

    public Arm(Logger aLogger, OpMode aOpMode, String crankName, String switch1Name, String switch2Name)
    {
        super(aLogger, aOpMode);

        // Define and Initialize Motors
        crank = initMotor(crankName, DcMotor.Direction.FORWARD);

        // guard switches disabled for now as not used
        //forwardGuardSwitch = initTouchSensor(switch1Name);
        //backwardGuardSwitch = initTouchSensor(switch2Name);

        if (crank != null) {
            isAvailable = true;
        }

        logger.logDebug("Arm","isAvailable:%b", isAvailable);
    }

    public void crankForward(double power) {
        while (opModeIsActive() && !forwardGuardSwitch.isPressed() && opMode.gamepad1.right_bumper) {
            crank.setDirection(DcMotorSimple.Direction.FORWARD);
            crank.setPower(power);
            idle();
        }
        crank.setPower(0.0);
    }

    public void crankBackward(double power) {
        while (opModeIsActive() && !backwardGuardSwitch.isPressed() && opMode.gamepad1.left_bumper) {
            crank.setDirection(DcMotorSimple.Direction.REVERSE);
            crank.setPower(power);
            idle();
        }
        crank.setPower(0.0);
    }

}

