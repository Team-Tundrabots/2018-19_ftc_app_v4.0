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
import com.qualcomm.robotcore.util.ElapsedTime;


public class DriveTrain extends BotComponent {

    public DcMotor frontLeftMotor = null;
    public DcMotor frontRightMotor = null;

    public DcMotor backLeftMotor = null;
    public DcMotor backRightMotor = null;
    
    /* Constructor */
    public DriveTrain() {

    }

    public DriveTrain(OpMode aOpMode, String frontLeftMotorName, String frontRightMotorName,
                                      String backLeftMotorName, String backRightMotorName)
    {
        super(aOpMode);

        // Define and Initialize Motors
        frontLeftMotor = initMotor(frontLeftMotorName, DcMotor.Direction.REVERSE);
        frontRightMotor = initMotor(frontRightMotorName);

        backLeftMotor = initMotor(backLeftMotorName, DcMotor.Direction.REVERSE);
        backRightMotor = initMotor(backRightMotorName);

    }

    public void setLeftMotorsPower(double power){
        if (frontLeftMotor != null) {
            frontLeftMotor.setPower(power);
        }
        if (backLeftMotor != null) {
            backLeftMotor.setPower(power);
        }
    }

    public void setRightMotorsPower(double power){
        if (frontRightMotor != null) {
            frontRightMotor.setPower(power);
        }
        if (backRightMotor != null) {
            backRightMotor.setPower(power);
        }
    }

    public void move(double seconds, double leftPower, double rightPower) {

        ElapsedTime runtime = new ElapsedTime();

        setLeftMotorsPower(leftPower);
        setRightMotorsPower(rightPower);

        runtime.reset();
        while(runtime.seconds() < seconds) {
            opMode.telemetry.addData("Path", "Time: %2.5f S Elapsed", runtime.seconds());
            opMode.telemetry.update();
        }
        setLeftMotorsPower(0.0);
        setRightMotorsPower(0.0);

    }

    public void moveForward(double seconds, double power)
    {
        move(seconds,power, power);
    }

    public void moveBackward(double seconds, double power)
    {
        // move forwards with negative power
        move(seconds, -power, -power);
    }

    public void turnLeft(double seconds, double power) {
        move(seconds, -power, power);
    }

    public void turnRight(double seconds, double power) {
        move(seconds, power, -power);
    }

    public void updateMotorsTankDrive(double leftY, double rightY) {

        double left;
        double right;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = -leftY;
        right = -rightY;

        setLeftMotorsPower(left);
        setRightMotorsPower(right);

    }

    public void updateMotorsMechanumDrive(double leftX, double leftY, double rightX, double rightY) {

        // reverse Y coordinates
        double lX = leftX;
        double lY = -leftY;
        double rX = rightX;
        double rY = -rightY;

        double r = Math.hypot(lX, lY);
        double robotAngle = Math.atan2(lY, lX) - Math.PI / 4;

        final double v1 = r * Math.cos(robotAngle) + rX;
        final double v2 = r * Math.sin(robotAngle) - rX;
        final double v3 = r * Math.sin(robotAngle) + rX;
        final double v4 = r * Math.cos(robotAngle) - rX;

        frontLeftMotor.setPower(v1);
        frontRightMotor.setPower(v2);
        backLeftMotor.setPower(v3);
        backRightMotor.setPower(v4);

    }
}

