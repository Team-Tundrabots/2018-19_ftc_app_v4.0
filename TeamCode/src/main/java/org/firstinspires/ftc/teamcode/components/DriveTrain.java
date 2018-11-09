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

    public void stop() {
        setLeftMotorsPower(0.0);
        setRightMotorsPower(0.0);
    }

    public void crabLeft(double seconds) {



        ElapsedTime runtime = new ElapsedTime();

        updateMotorsMechanumDrive(-1, 0, 0, -1);

        runtime.reset();
        while(runtime.seconds() < seconds) {
            opMode.telemetry.addData("Path", "Time: %2.5f S Elapsed", runtime.seconds());
            opMode.telemetry.update();
        }
        setLeftMotorsPower(0.0);
        setRightMotorsPower(0.0);

    }

    public void crabRight(double seconds) {



        ElapsedTime runtime = new ElapsedTime();

        updateMotorsMechanumDrive(0, -1, 1, 0);

        runtime.reset();
        while(runtime.seconds() < seconds) {
            opMode.telemetry.addData("Path", "Time: %2.5f S Elapsed", runtime.seconds());
            opMode.telemetry.update();
        }
        setLeftMotorsPower(0.0);
        setRightMotorsPower(0.0);

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

    public void resetEncoders() {
        // Send telemetry message to signify robot waiting;
        opMode.telemetry.addData("Status", "Resetting Encoders");    //
        opMode.telemetry.update();

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        opMode.telemetry.addData("Path0",  "Starting at %7d :%7d",
                frontLeftMotor.getCurrentPosition(),
                frontRightMotor.getCurrentPosition());
        opMode.telemetry.update();

    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {

        double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
        double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
        double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
        double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);
        double     DRIVE_SPEED             = 0.6;
        double     TURN_SPEED              = 0.5;


        int newLeftTarget;
        int newRightTarget;

        ElapsedTime runtime = new ElapsedTime();

        // Ensure that the opmode is still active - ToDo: Need to pull this from the opmode
        boolean opModeIsActive = true;

        if (opModeIsActive) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = frontLeftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = frontRightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            frontLeftMotor.setTargetPosition(newLeftTarget);
            frontRightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            frontLeftMotor.setPower(Math.abs(speed));
            frontRightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeftMotor.isBusy() && frontRightMotor.isBusy())) {

                // Display it for the driver.
                opMode.telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                opMode.telemetry.addData("Path2",  "Running at %7d :%7d",
                        frontLeftMotor.getCurrentPosition(),
                        frontRightMotor.getCurrentPosition());
                opMode.telemetry.update();
            }

            // Stop all motion;
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
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

    public void encoderDrive2(double Lspeed, double Rspeed, double Inches, double timeoutS, double rampup) throws InterruptedException {

        double     COUNTS_PER_MOTOR_REV    = 560 ;    //Set for NevRest 20 drive. For 40's change to 1120. For 60's 1680
        double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is the ratio between the motor axle and the wheel
        double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
        double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);

        ElapsedTime runtime = new ElapsedTime();

        //initialise some variables for the subroutine
        int newLeftTarget;
        int newRightTarget;
        // Ensure that the opmode is still active
        // Determine new target position, and pass to motor controller we only do this in case the encoders are not totally zero'd

        // Determine new target position, and pass to motor controller
        newLeftTarget = (getFrontLeftPosition()+ getBackLeftPosition() )/2 + (int)(Inches * COUNTS_PER_INCH);
        newRightTarget = (getFrontRightPosition() + getBackRightPosition() )/2 + (int)(Inches * COUNTS_PER_INCH);

        // reset the timeout time and start motion.
        runtime.reset();

        // keep looping while we are still active, and there is time left, and neither set of motors have reached the target
        while ( (runtime.seconds() < timeoutS) &&
                (Math.abs(getFrontLeftPosition()+ getBackLeftPosition()) /2 < newLeftTarget  &&
                        Math.abs(getFrontRightPosition() + getBackRightPosition())/2 < newRightTarget)) {
            double rem = (Math.abs(getFrontLeftPosition()) + Math.abs(getBackLeftPosition())+Math.abs(getFrontRightPosition()) + Math.abs(getBackRightPosition()))/4;
            double NLspeed;
            double NRspeed;
            //To Avoid spinning the wheels, this will "Slowly" ramp the motors up over
            //the amount of time you set for this SubRun
            double R = runtime.seconds();
            if (R < rampup) {
                double ramp = R / rampup;
                NLspeed = Lspeed * ramp;
                NRspeed = Rspeed * ramp;
            }

            //Keep running until you are about two rotations out
            else if(rem > (1000) )
            {
                NLspeed = Lspeed;
                NRspeed = Rspeed;
            }
            //start slowing down as you get close to the target
            else if(rem > (200) && (Lspeed*.2) > .1 && (Rspeed*.2) > .1) {
                NLspeed = Lspeed * (rem / 1000);
                NRspeed = Rspeed * (rem / 1000);
            }
            //minimum speed
            else {
                NLspeed = Lspeed * .2;
                NRspeed = Rspeed * .2;

            }
            //Pass the seed values to the motors
            frontLeftMotor.setPower(NLspeed);
            backLeftMotor.setPower(NLspeed);
            frontRightMotor.setPower(NRspeed);
            backRightMotor.setPower(NRspeed);
        }

        // Stop all motion;
        //Note: This is outside our while statement, this will only activate once the time, or distance has been met
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        // show the driver how close they got to the last target
        opMode.telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
        opMode.telemetry.addData("Path2",  "Running at %7d :%7d", getFrontLeftPosition(), getFrontRightPosition());
        opMode.telemetry.update();
/*
        //setting resetC as a way to check the current encoder values easily
        double resetC = ((Math.abs(getFrontLeftPosition()) + Math.abs(getFrontRightPosition())+ Math.abs(getFrontRightPosition())+Math.abs(getFrontRightPosition())));
        //Get the motor encoder resets in motion
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //keep waiting while the reset is running
        while (Math.abs(resetC) > 0){
            resetC =  ((Math.abs(getFrontLeftPosition()) + Math.abs(getFrontRightPosition())+ Math.abs(getFrontRightPosition())+Math.abs(getFrontRightPosition())));
            pause(.25);
        }
*/
        // switch the motors back to RUN_USING_ENCODER mode
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //give the encoders a chance to switch modes.
        pause(.25);
    }

}

