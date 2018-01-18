package org.firstinspires.ftc.teamcode.mecanum;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by USER on 1/3/2018.
 */

@TeleOp(name="alignLift", group="Temp")
public class alignLift extends MecanumOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        waitForStart();

        telemetry.log().add("started");

        while (opModeIsActive()) {
            if (gamepad1.left_stick_y == 0) {
                leftLift.setPower(gamepad2.left_stick_y);
                rightLift.setPower(gamepad2.right_stick_y);
            } else {
                leftLift.setPower(gamepad1.left_stick_y);
                rightLift.setPower(gamepad1.left_stick_y);
            }
        }
    }
}
