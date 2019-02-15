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


public class Hoist extends BotComponent {

    public DcMotor crank = null;
    public TouchSensor guardSwitch = null;

    private int ROTATIONS_PER_INCH = 4400;

    private int DEFAULT_EXTENDED_POSITION = 3000;
    private int DEFAULT_CONTRACTED_POSITION = 0;
    private int DEFAULT_RAMP_UP_DOWN_THRESHOLD = 200;
    private double DEFAULT_POWER = 0.25;

    public double extendedPositionInches = DEFAULT_EXTENDED_POSITION * ROTATIONS_PER_INCH;
    public double contractedPositionInches = DEFAULT_CONTRACTED_POSITION * ROTATIONS_PER_INCH;

    public int extendedPosition = DEFAULT_EXTENDED_POSITION;
    public int contractedPosition = DEFAULT_CONTRACTED_POSITION;
    public int rampUpDownThreshold = DEFAULT_RAMP_UP_DOWN_THRESHOLD;
    public double power = DEFAULT_POWER;
    private boolean isGuardSwitchEnabled = true;

    /* Constructor */
    public Hoist() {

    }

    public Hoist(Logger aLogger, OpMode aOpMode, String crankName) {
        super(aLogger, aOpMode);
        crank = initMotor(crankName, DcMotor.Direction.FORWARD, true);
        init();
    }

    public void outputCrankPositions() {
        outputCrankPositions("Crank Positions");
    }

    public void outputCrankPositions(String label) {
        logger.logDebug("outputCrankPositions", "Status: %s, Target: %7d, Current: %7d, Power: %f", label, crank.getTargetPosition(), crank.getCurrentPosition(), crank.getPower());
    }

    private void init() {
        try {
            crank.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            crank.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            isAvailable = true;
        } catch (NullPointerException err) {
            logger.logErr("Host.init","Error", err.getMessage());
        }
        logger.logInfo("Hoist.init","isAvailable:%b", isAvailable);

    }

    public void setTargetInches(double inches) {
        logger.logDebug("setTargetInches", "inches * ROTATIONS_PER_INCH=%f", inches * ROTATIONS_PER_INCH);
        setTarget((int) inches * ROTATIONS_PER_INCH);
    }

    public void setTarget(int targetPosition) {
        crank.setTargetPosition(targetPosition);
        crank.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outputCrankPositions();
    }

    public void extend() {
        logger.logDebug("Hoist.extend", "");
        crank.setDirection(DcMotorSimple.Direction.REVERSE);
        runToTarget(extendedPosition, power, rampUpDownThreshold);
    }

    public void contract() {
        logger.logDebug("Hoist.contract", "");
        crank.setDirection(DcMotorSimple.Direction.FORWARD);
        runToTarget(contractedPosition, power, rampUpDownThreshold);
    }

    private void runToTarget(int targetPosition, double power, int rampUpDownThreshold) {
        int startPosition = crank.getCurrentPosition();
        setTarget(targetPosition);
        crank.setPower(power);

        double calcPower = 0;

        while (opModeIsActive() && (crank.isBusy() || opMode.gamepad1.x)) {

            int diffStart = Math.abs(startPosition - crank.getCurrentPosition()) + 1;
            int diffTarget = Math.abs(targetPosition - crank.getCurrentPosition()) + 1;

            if (!opMode.gamepad1.x) {
                crank.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (diffTarget < rampUpDownThreshold && calcPower > .01) {
                    calcPower = power * ((double) diffTarget / rampUpDownThreshold); // calcPower - .01;
                    logger.logDebug("outputCrankPositions", "Status: %s, diffTarget: %7d, rampUpDownThreshold: %7d, Power: %f", "Hoist.runToTarget:Ramp Down", diffTarget, rampUpDownThreshold, calcPower);
                    crank.setPower(calcPower);
                    outputCrankPositions("Hoist.runToTarget:Ramp Down");
                } else if (diffStart < rampUpDownThreshold && calcPower < power) {
                    calcPower = power * ((double) diffStart / rampUpDownThreshold) + 0.01; // calcPower + .01;
                    logger.logDebug("outputCrankPositions", "Status: %s, diffStart: %7d, rampUpDownThreshold: %7d, Power: %f", "Hoist.runToTarget:Ramp Up", diffStart, rampUpDownThreshold, calcPower);
                    crank.setPower(calcPower);
                    outputCrankPositions("Hoist.runToTarget:Ramp Up");
                } else {
                    crank.setPower(power);
                    outputCrankPositions("Hoist.runToTarget:Running");
                }
            } else {
                crank.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                crank.setPower(power);
                outputCrankPositions("Hoist.runToTarget:OVERRIDE");
            }

            idle();

        }

        crank.setPower(0);
        // Turn off RUN_TO_POSITION
        crank.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}



