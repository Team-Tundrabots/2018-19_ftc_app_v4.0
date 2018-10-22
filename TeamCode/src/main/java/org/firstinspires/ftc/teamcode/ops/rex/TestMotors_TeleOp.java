

package org.firstinspires.ftc.teamcode.ops.rex;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bots.CompetitionBot;


@TeleOp(name="TestMotors_TeleOp", group="rex")
//@Disabled

public class TestMotors_TeleOp extends OpMode{

    /* Declare OpMode members. */
    CompetitionBot robot       = new CompetitionBot(this);

    /*
     * Code to run ONCE when the driver hits INIT
     */


    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        robot.driveTrain.pause(2);

        if(gamepad1.y){
            robot.driveTrain.frontLeftMotor.setPower(1);
            telemetry.addData("frontLeftMotor", "1");
        } else if(gamepad1.b){
            robot.driveTrain.frontRightMotor.setPower(1);
            telemetry.addData("frontRightMotor", "1");
        } else if(gamepad1.x){
            robot.driveTrain.backLeftMotor.setPower(1);
            telemetry.addData("backLeftMotor", "1");
        } else if (gamepad1.a) {
            robot.driveTrain.backRightMotor.setPower(1);
            telemetry.addData("backRightMotor", "1");
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
