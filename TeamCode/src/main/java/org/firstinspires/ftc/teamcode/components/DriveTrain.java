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

import org.firstinspires.ftc.teamcode.bots.*;

public class DriveTrain extends BotComponent {

    public DcMotor frontLeftMotor = null;
    public DcMotor frontRightMotor = null;

    public DcMotor backLeftMotor = null;
    public DcMotor backRightMotor = null;

    public GyroNavigator gyroNavigator = null;


    private double COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    private double DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private double WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                             (WHEEL_DIAMETER_INCHES * 3.1415);


    /* Constructor */
    public DriveTrain() {

    }

    public DriveTrain(Logger aLogger, OpMode aOpMode,
                String frontLeftMotorName, String frontRightMotorName,
                String backLeftMotorName, String backRightMotorName)
    {
        super(aLogger, aOpMode);

        // Define and Initialize Motors
        frontLeftMotor = initMotor(frontLeftMotorName, DcMotor.Direction.REVERSE);
        frontRightMotor = initMotor(frontRightMotorName);

        backLeftMotor = initMotor(backLeftMotorName, DcMotor.Direction.REVERSE);
        backRightMotor = initMotor(backRightMotorName);

        // as long as a left and right motor are configured - assume DriveTrain is available
        if (((frontLeftMotor != null) && (frontRightMotor != null)) ||
            ((backLeftMotor !=null) && (backLeftMotor !=null))) {
            isAvailable = true;
        }

        logger.logDebug("DriveTrain","isAvailable:%b", isAvailable);

    }

    public DriveTrain(Logger aLogger, OpMode aOpMode,
                      String frontLeftMotorName, String frontRightMotorName,
                      String backLeftMotorName, String backRightMotorName,
                      GyroNavigator aGyroNavigator) {
        this(aLogger, aOpMode, frontLeftMotorName, frontRightMotorName, backLeftMotorName, backRightMotorName);

        if (this.isAvailable ) {
            gyroNavigator = aGyroNavigator;
        }
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
        while(runtime.seconds() < seconds && opModeIsActive()) {
            opMode.telemetry.addData("Path", "Time: %2.5f S Elapsed", runtime.seconds());
            opMode.telemetry.update();
        }
        stop();

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

    public void stop() {
        setLeftMotorsPower(0.0);
        setRightMotorsPower(0.0);
    }

    public void crabLeft(double seconds) {

        ElapsedTime runtime = new ElapsedTime();

        // leftX -1, rightY -1
        updateMotorsMechanumDrive(-1, 0, 0, -1);

        runtime.reset();
        while(runtime.seconds() < seconds && opModeIsActive()) {
            opMode.telemetry.addData("Path", "Time: %2.5f S Elapsed", runtime.seconds());
            opMode.telemetry.update();
        }
        stop();
    }

    public void crabRight(double seconds) {

        ElapsedTime runtime = new ElapsedTime();

        // crabRight = lefty -1, rightX 1
        updateMotorsMechanumDrive(1, 0, 0, -1);

        runtime.reset();
        while(runtime.seconds() < seconds && opModeIsActive()) {
            opMode.telemetry.addData("Path", "Time: %2.5f S Elapsed", runtime.seconds());
            opMode.telemetry.update();
        }
        stop();
    }

    public void updateMotorsMechanumDrive(double leftX, double leftY, double rightX, double rightY) {


        double lX = -leftX;
        double lY = leftY;
        double rX = -rightX;
        double rY = rightY;

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

        if (!opModeIsActive()) { stop(); }

    }


    public void disableEncoders() {

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void resetEncoders() {

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        logger.logDebug("resetEncoders",  "Front:  Left:%7d Right:%7d", getFrontLeftPosition(), getFrontRightPosition());
        logger.logDebug("resetEncoders",  "Back:   Left:%7d Right:%7d", getBackLeftPosition(), getBackRightPosition());

    }

    private void setTargetPositions(int leftTarget, int rightTarget) {

        frontLeftMotor.setTargetPosition(leftTarget);
        frontRightMotor.setTargetPosition(rightTarget);
        backLeftMotor.setTargetPosition(leftTarget);
        backRightMotor.setTargetPosition(rightTarget);

        // Turn On RUN_TO_POSITION
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double power,
                             double leftInches, double rightInches,
                             double timeoutSeconds) {

        int newLeftTarget;
        int newRightTarget;

        resetEncoders();

        ElapsedTime runtime = new ElapsedTime();

        // Ensure that the opmode is still active - ToDo: Need to pull this from the opmode
        // boolean opModeIsActive = true;

        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = backLeftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = backRightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            setTargetPositions(newLeftTarget, newRightTarget);

            // reset the timeout time and start motion.
            runtime.reset();

//            frontLeftMotor.setPower(power);
//            frontRightMotor.setPower(power);
            backLeftMotor.setPower(power);
            backRightMotor.setPower(power);

            //setLeftMotorsPower(Math.abs(power));
            //setRightMotorsPower(Math.abs(power));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutSeconds) &&
                    (backLeftMotor.isBusy() && backRightMotor.isBusy())) {

                logger.logDebug("encoderDrive", "Target: Left:%7d Right:%7d", newLeftTarget,  newRightTarget);
                logger.logDebug("encoderDrive", "Front:  Left:%7d Right:%7d", getFrontLeftPosition(), getFrontRightPosition());
                logger.logDebug("encoderDrive", "Back:   Left:%7d Right:%7d", getBackLeftPosition(), getBackRightPosition());
                logger.logDebug("encoderDrive", "runtime.seconds: %f", runtime.seconds());

                opMode.telemetry.update();
            }

            // Stop all motion;
            stop();

            disableEncoders();

        }
        
        
    }

    public void gyroEncoderDrive(double power,
                                 double leftInches, double rightInches,
                                 double timeoutSeconds) {

        // keep the current angle as the target to stay on
        int targetAngle = (int) gyroNavigator.getAngle();

        int newLeftTarget;
        int newRightTarget;

        resetEncoders();

        ElapsedTime runtime = new ElapsedTime();

        // Ensure that the opmode is still active - ToDo: Need to pull this from the opmode
        // boolean opModeIsActive = true;

        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = backLeftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = backRightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            setTargetPositions(newLeftTarget, newRightTarget);

            // reset the timeout time and start motion.
            runtime.reset();


//            frontLeftMotor.setPower(power);
//            frontRightMotor.setPower(power);
            backLeftMotor.setPower(power);
            backRightMotor.setPower(power);

            //setLeftMotorsPower(Math.abs(power));
            //setRightMotorsPower(Math.abs(power));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutSeconds) &&
                    (backLeftMotor.isBusy() && backRightMotor.isBusy())) {

                logger.logDebug("encoderDrive", "Target: Left:%7d Right:%7d", newLeftTarget,  newRightTarget);
                logger.logDebug("encoderDrive", "Front:  Left:%7d Right:%7d", getFrontLeftPosition(), getFrontRightPosition());
                logger.logDebug("encoderDrive", "Back:   Left:%7d Right:%7d", getBackLeftPosition(), getBackRightPosition());
                logger.logDebug("encoderDrive", "runtime.seconds: %f", runtime.seconds());

                if (Math.abs(gyroNavigator.getAngle()-targetAngle) > 2) {
                    gyroRotate(targetAngle, power);
                }

                opMode.telemetry.update();
            }

            // Stop all motion;
            stop();

            disableEncoders();

        }


    }

    private int getFrontLeftPosition() {
        return frontLeftMotor.getCurrentPosition();        
    }

    private int getFrontRightPosition() {
        return frontRightMotor.getCurrentPosition();
    }

    private int getBackLeftPosition() {
        return backLeftMotor.getCurrentPosition();
    }

    private int getBackRightPosition() {
        return backRightMotor.getCurrentPosition();
    }

    /***
     * rotate using gyro
      * @param degrees
     * @param power
     */
    public void gyroRotate(int degrees, double power) {

        if (!gyroNavigator.isAvailable) {
            logger.logErr("rotate", "gyroNavigator is not available");
            return;
        }

        double currentAngle = gyroNavigator.getAngle();
        double targetAngle = currentAngle + degrees;


        boolean rotationComplete = false;
        while (opModeIsActive() && !rotationComplete) {

            logger.logDebug("gyroRotate", "degrees:%d, power:%f", degrees, power);
            double leftPower = 0;
            double rightPower = 0;

            if (degrees < 0)
            {   // turn left.
                logger.logDebug("gyroRotate", "turning left");
                leftPower = power;
                rightPower = - power;
                if (currentAngle <= targetAngle) {
                    rotationComplete = true;
                }
            } else if (degrees > 0) {   // turn right.
                logger.logDebug("gyroRotate", "turning right");
                leftPower = - power;
                rightPower = power;
                if (currentAngle >= targetAngle) {
                    rotationComplete = true;
                }
            } else {
                rotationComplete = true;
            }

            currentAngle = gyroNavigator.getAngle();
            logger.logDebug("gyroRotate", "currentAngle:%f, targetAngle:%f", currentAngle, targetAngle);

            logger.logDebug("gyroRotate", "rotationComplete:%b", rotationComplete);
            if (!rotationComplete) {
                setLeftMotorsPower(leftPower);
                setRightMotorsPower(rightPower);
            } else {
                stop();
            }

            opMode.telemetry.update();
            idle();

        }
    }

}

