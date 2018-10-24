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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class BotComponent {

    public OpMode opMode = null;

    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public BotComponent() {

    }

    public BotComponent(OpMode aOpMode) {
        opMode = aOpMode;
    }

    private LinearOpMode getLinearOpMode() throws ClassCastException {
        LinearOpMode op = (LinearOpMode) opMode;
        return op;
    }

    public boolean opModeIsActive() {
        LinearOpMode op = null;
        try {
            op = getLinearOpMode();
        } catch (ClassCastException err) {
            return true;
        }
        return op.opModeIsActive();
    }

    public DcMotor initMotor(String motorName) {
        return(initMotor(motorName, DcMotor.Direction.FORWARD));
    }

    public DcMotor initMotor(String motorName, DcMotorSimple.Direction direction) {

        try {
            HardwareMap ahwMap = opMode.hardwareMap;
            DcMotor motor = ahwMap.get(DcMotor.class, motorName);

            motor.setDirection(direction);
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            return (motor);

        } catch (NullPointerException | IllegalArgumentException err) {
            if (opMode.telemetry != null) {
                opMode.telemetry.addData("Error", err.getMessage());
            }
            return null;
        }
    }

    public Servo initServo(String servoName, double position) {
        try {
            HardwareMap ahwMap = opMode.hardwareMap;
            Servo servo = ahwMap.get(Servo.class, servoName);
            servo.setPosition(position);

            return (servo);

        } catch (NullPointerException | IllegalArgumentException err) {
            if (opMode.telemetry != null) {
                opMode.telemetry.addData("Error", err.getMessage());
            }
            return null;
        }
    }

    public void pause(double seconds) {
        long milliseconds = (long)seconds * 1000;

        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

    }

}

