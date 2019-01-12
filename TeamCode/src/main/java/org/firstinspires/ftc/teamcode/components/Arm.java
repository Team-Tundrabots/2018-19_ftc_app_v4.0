



package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

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

        // get a reference to our digitalTouch object.
        forwardGuardSwitch = initTouchSensor(switch1Name);

        backwardGuardSwitch = initTouchSensor(switch2Name);

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


