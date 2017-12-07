package org.firstinspires.ftc.teamcode.temp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by USER on 11/21/2017.
 */

@TeleOp(name="collector", group="Temp")
public class TestCollector extends TempOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        waitForStart();

        double power = -0.25;

        while (opModeIsActive()) {

            if (gamepad1.y)
                power = gamepad1.left_stick_y;

            if (gamepad1.a) {
                leftFront.setPower(power);
                rightFront.setPower(-power);
            } else if (gamepad1.b) {
                leftFront.setPower(-power);
                rightFront.setPower(power);
            } else if (gamepad1.x) {
                leftFront.setPower(0);
                rightFront.setPower(0);
            } else {
                leftFront.setPower(power);
                rightFront.setPower(power);
            }

            telemetry.addData("power", "$.2f", power);

            telemetry.update();
        }
    }
}
