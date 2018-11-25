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
import com.qualcomm.robotcore.util.ElapsedTime;


public class TestComponent extends BotComponent {

    public DcMotor testMotor = null;
    public TouchSensor testSwitch = null;

    private ElapsedTime runtime = new ElapsedTime();

    /* Constructor */
    public TestComponent() {

    }

    public TestComponent(OpMode aOpMode, String testMotorName, String testSwitchName)
    {
        super(aOpMode);

        // Define and Initialize Motors
        testMotor = initMotor(testMotorName, DcMotor.Direction.FORWARD, true);

        // get a reference to our digitalTouch object.
        testSwitch = initTouchSensor(testSwitchName);

    }


    public void reset() {
        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        opMode.telemetry.addData("Start:", testMotor.getCurrentPosition());
        opMode.telemetry.update();

    }

    public void setTarget(int targetPosition) {
        testMotor.setTargetPosition(targetPosition);
        // Turn On RUN_TO_POSITION
        testMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void runToTarget(int targetPosition, double power, double timeoutS) {
        testMotor.setTargetPosition(targetPosition);

        // Turn On RUN_TO_POSITION
        testMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        testMotor.setPower(Math.abs(power));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (testMotor.isBusy())) {

            // Display it for the driver.
            opMode.telemetry.addData("Path1",  "Running to %7d", targetPosition);
            opMode.telemetry.addData("Path2",  "Running at %7d",
                    testMotor.getCurrentPosition());
            opMode.telemetry.update();
        }

        // Stop all motion;
        testMotor.setPower(0);

        // Turn off RUN_TO_POSITION
        testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

}

