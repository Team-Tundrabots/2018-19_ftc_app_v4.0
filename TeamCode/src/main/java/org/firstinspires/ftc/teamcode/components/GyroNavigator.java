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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class GyroNavigator extends BotComponent {

    private DriveTrain driveTrain = null;
    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double globalAngle, power = .30, correction;

    /* Constructor */
    public GyroNavigator() {

    }

    public GyroNavigator(Logger aLogger, OpMode aOpMode, DriveTrain aDriveTrain)
    {
        super(aLogger, aOpMode);

        driveTrain = aDriveTrain;

    }

    public void init() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        opMode.telemetry.addData("Mode", "calibrating...");
        opMode.telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (opModeIsActive() && !imu.isGyroCalibrated())
        {
            driveTrain.pause(.5);
            driveTrain.idle();
        }

        isAvailable = true;

        opMode.telemetry.addData("Mode", "waiting for start");
        opMode.telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());


    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. - = left, + = right.
     */
    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return 0 - globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    public void rotate(int degrees, double power) {
        double currentAngle = getAngle();
        double targetAngle = currentAngle + degrees;


        boolean rotationComplete = false;
        while (opModeIsActive() && !rotationComplete) {

            logger.logDebug("rotate", "degrees:%d, power:%f", degrees, power);
            double leftPower = 0;
            double rightPower = 0;

            if (degrees < 0)
            {   // turn left.
                logger.logDebug("rotate", "turning left");
                leftPower = power;
                rightPower = - power;
                if (currentAngle <= targetAngle) {
                    rotationComplete = true;
                }
            } else if (degrees > 0) {   // turn right.
                logger.logDebug("rotate", "turning right");
                leftPower = - power;
                rightPower = power;
                if (currentAngle >= targetAngle) {
                    rotationComplete = true;
                }
            } else {
                rotationComplete = true;
            }

            currentAngle = getAngle();
            logger.logDebug("rotate", "currentAngle:%f, targetAngle:%f", currentAngle, targetAngle);

            logger.logDebug("rotate", "rotationComplete:%b", rotationComplete);
            if (!rotationComplete) {
                driveTrain.setLeftMotorsPower(leftPower);
                driveTrain.setRightMotorsPower(rightPower);
            } else {
                driveTrain.stop();
            }

            opMode.telemetry.update();
            idle();

        }
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotate2(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns - when rotating counter clockwise (left) and + when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn left.
            leftPower = - power;
            rightPower = power;

        }
        else if (degrees > 0)
        {   // turn right.
            leftPower = power;
            rightPower = - power;
        }
        else return;

        // set power to rotate.
        driveTrain.setLeftMotorsPower(leftPower);
        driveTrain.setRightMotorsPower(rightPower);
        opMode.telemetry.addData("angle",getAngle());
        opMode.telemetry.addData("degrees",degrees);
        opMode.telemetry.update();

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
                opMode.telemetry.addData("angle",getAngle());
                opMode.telemetry.addData("degrees",degrees);
                opMode.telemetry.update();
            }

            while (opModeIsActive() && getAngle() > degrees) {
                opMode.telemetry.addData("angle",getAngle());
                opMode.telemetry.addData("degrees",degrees);
                opMode.telemetry.update();

            }
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {
                opMode.telemetry.addData("angle",getAngle());
                opMode.telemetry.addData("degrees",degrees);
                opMode.telemetry.update();
            }

        // turn the motors off.
        driveTrain.stop();

        // wait for rotation to stop.
        driveTrain.pause(1);

        // reset angle tracking on new heading.
        resetAngle();
    }




}