package org.firstinspires.ftc.teamcode.mecanum;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Motor8696;

@TeleOp(name="testNewBot", group="Temp")
public class NewRobot extends MecanumOpMode {

    /**
     * {@inheritDoc}
     */
    @Override
    void buttonEvents() {
        super.buttonEvents();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftLift.setTargetPosition(0);
        rightLift.setTargetPosition(0);

        initGyro();
        buttonEvents();

        waitForStart();
        ballPoosher.setPosition(0);

        telemetry.log().clear();

        while (opModeIsActive()) {
            runCollector();

            runLift();
            alignLift();

            for (Motor8696 motor : motors) {
                telemetry.addData(motor.getPortNumber() + "", motor.getCurrentPosition());
            }

            if (!dpadDrive()) {
                if (reverse > 0)
                    verticalDrive(reverse * driveSpeed, gamepad1.left_stick_y, gamepad1.right_stick_y);
                else
                    verticalDrive(reverse * driveSpeed, gamepad1.right_stick_y, gamepad1.left_stick_y);
                horizontalDrive(reverse * driveSpeed * (gamepad1.right_trigger - gamepad1.left_trigger));

                for (Motor8696 motor : motors) {
                    motor.setMaxPower(Math.sqrt(2));
                }

                runMotors();
            }

            buttonEvents.run();
            telemetry.update();
        }
    }
}
