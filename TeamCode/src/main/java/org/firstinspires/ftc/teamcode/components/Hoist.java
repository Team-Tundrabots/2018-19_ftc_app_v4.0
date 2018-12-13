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


public class Hoist extends BotComponent {

    public DcMotor crank = null;
    public TouchSensor guardSwitch = null;

    private int DEFAULT_LOWERED_POSITION = 3000;
    private int DEFAULT_RAISED_POSITION = 0;
    private int DEFAULT_RAMP_UP_DOWN_THRESHOLD = 200;
    private double DEFAULT_POWER = 0.25;

    public int loweredPosition = DEFAULT_LOWERED_POSITION;
    public int raisedPosition = DEFAULT_RAISED_POSITION;
    public int rampUpDownThreshold = DEFAULT_RAMP_UP_DOWN_THRESHOLD;
    public double power = DEFAULT_POWER;

    /* Constructor */
    public Hoist() {

    }

    public Hoist(Logger aLogger, OpMode aOpMode, String crankName, String guardSwitchName)
    {
        super(aLogger, aOpMode);

        // Define and Initialize Motors
        crank = initMotor(crankName, DcMotor.Direction.FORWARD, true);

        // get a reference to our digitalTouch object.
        guardSwitch = initTouchSensor(guardSwitchName);

        reset();

    }

    public void outputCrankPositions() {
        outputCrankPositions("Crank Positions");
    }

    public void outputCrankPositions(String label) {
        logger.logDebug("outputCrankPositions", "Status: %s, Target: %7d, Current: %7d, Power: %f", label, crank.getTargetPosition(), crank.getCurrentPosition(), crank.getPower());
        opMode.telemetry.addData("Status", label);
        opMode.telemetry.addData("Target",  "%7d", crank.getTargetPosition());
        opMode.telemetry.addData("Current", "%7d", crank.getCurrentPosition());
        opMode.telemetry.addData("Power", crank.getPower());
        opMode.telemetry.update();
    }

    public void reset() {
        crank.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        crank.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outputCrankPositions("Hoist.reset");
    }

    public void setTarget(int targetPosition) {
        crank.setTargetPosition(targetPosition);
        crank.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outputCrankPositions();
    }

    public void lower() {
        logger.logDebug("Hoist.lower", "");
        crank.setDirection(DcMotorSimple.Direction.FORWARD);
        runToTarget(loweredPosition, power, rampUpDownThreshold);
    }

    public void raise() {
        logger.logDebug("Hoist.raise", "");
        crank.setDirection(DcMotorSimple.Direction.REVERSE);
        runToTarget(raisedPosition, power,  rampUpDownThreshold);
    }

    // Simple version - no ramp up or down
    private void runToTarget(int targetPosition, double power) {
        int startPosition = crank.getCurrentPosition();
        setTarget(targetPosition);
        crank.setPower(power);
        while (opModeIsActive() && crank.isBusy() && !guardSwitch.isPressed()) {
            outputCrankPositions("Hoist.runToTarget");
            idle();
        }
        crank.setPower(0);
        // Turn off RUN_TO_POSITION
        crank.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void runToTarget(int targetPosition, double power, int rampUpDownThreshold) {
        int startPosition = crank.getCurrentPosition();
        setTarget(targetPosition);
        crank.setPower(power);

        double calcPower = 0;

        while (opModeIsActive() && crank.isBusy() && !guardSwitch.isPressed()) {

            int diffStart = Math.abs(startPosition - crank.getCurrentPosition()) + 1;
            int diffTarget = Math.abs(targetPosition - crank.getCurrentPosition()) + 1;

            if ( diffTarget < rampUpDownThreshold && calcPower > .01) {
                calcPower = power * ((double)diffTarget / rampUpDownThreshold ); // calcPower - .01;
                logger.logDebug("outputCrankPositions", "Status: %s, diffTarget: %7d, rampUpDownThreshold: %7d, Power: %f", "Hoist.runToTarget:Ramp Down", diffTarget, rampUpDownThreshold, calcPower);
                crank.setPower(calcPower);
                outputCrankPositions("Hoist.runToTarget:Ramp Down");
            } else if ( diffStart < rampUpDownThreshold && calcPower < power) {
                calcPower = power * ((double)diffStart / rampUpDownThreshold) + 0.01; // calcPower + .01;
                logger.logDebug("outputCrankPositions", "Status: %s, diffStart: %7d, rampUpDownThreshold: %7d, Power: %f", "Hoist.runToTarget:Ramp Up", diffStart, rampUpDownThreshold, calcPower);
                crank.setPower(calcPower);
                outputCrankPositions("Hoist.runToTarget:Ramp Up");
            } else {
                crank.setPower(power);
                outputCrankPositions("Hoist.runToTarget:Running");
            }

            idle();
        }
        crank.setPower(0);
        // Turn off RUN_TO_POSITION
        crank.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runToTarget(int targetPosition, double power, double timeoutSeconds) {

        int startPosition = crank.getCurrentPosition();
        setTarget(targetPosition);
        double setPower = power;

        // reset the timeout time and start motion.
        //runtime.reset();
        crank.setPower(power);

        while (opModeIsActive()
                && (crank.isBusy())
//                && (runtime.seconds() < timeoutSeconds)
//                && (!guardSwitch.isPressed())
                ) {

            int diffStart = Math.abs(startPosition - crank.getCurrentPosition()) + 1;
            int diffTarget = Math.abs(targetPosition - crank.getCurrentPosition()) + 1;
/*
            if ( diffTarget < rampUpDownThreshold ) {
                setPower = DEFAULT_RAMP_DOWN_POWER;
                crank.setPower(setPower);
                outputCrankPositions("Ramp Down");
            } else if ( diffStart < rampUpDownThreshold ) {
                setPower = DEFAULT_RAMP_UP_POWER;
                crank.setPower(setPower);
                outputCrankPositions("Ramp Up");
            } else {
                setPower = power;
                crank.setPower(setPower);
                outputCrankPositions("Running");
            }
*/
            outputCrankPositions("Crank Running");
            idle();
        }

        // Stop all motion;
        crank.setPower(0);
        //outputCrankPositions();

        // Turn off RUN_TO_POSITION
        crank.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void waitForSwitch() {
        while (opModeIsActive() && !guardSwitch.isPressed()) {
            idle();
        }
    }

}
