package org.firstinspires.ftc.teamcode.mecanum;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by USER on 1/7/2018.
 */

@Disabled
@TeleOp(name="FancyDriving", group="Temp")
public class FancyDriving extends MecanumOpMode {

    @Override
    void buttonEvents() {
        super.buttonEvents();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        buttonEvents();

        waitForStart();


        double power = 0.35;

        while (opModeIsActive()) {

            getGyroData();

            runCollector();

//            leftLift.setPower(gamepad2.left_stick_y);
//            rightLift.setPower(gamepad2.left_stick_y);

            runLift();

            telemetry.addData("angles", angles.toString());

            ballPoosher.setPosition(gamepad2.left_trigger);

            telemetry.addData("gamepadLeft", getGamepadAngle(gamepad1.left_stick_x, gamepad1.left_stick_y));
            telemetry.addData("gamepadRight", getGamepadAngle(gamepad1.right_stick_x, gamepad1.right_stick_y));

            if (!dpadDrive()) {

                mecanumTeleOpDrive();
//                if (reverse > 0)
//                    verticalDrive(reverse * driveSpeed, gamepad1.left_stick_y, gamepad1.right_stick_y);
//                else
//                    verticalDrive(reverse * driveSpeed, gamepad1.right_stick_y, gamepad1.left_stick_y);
//                horizontalDrive(reverse * driveSpeed, (gamepad1.right_trigger - gamepad1.left_trigger));
//
//                for (Motor8696 motor : motors) {
//                    motor.setMaxPower(Math.sqrt(2));
//                }

//                runMotors();
            }

            buttonEvents.run();
            periodic(100);

            telemetry.update();
        }
    }
}
