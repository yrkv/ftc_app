package org.firstinspires.ftc.teamcode.mecanum;

import com.qualcomm.robotcore.eventloop.SyncdDevice;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Button;
import org.firstinspires.ftc.teamcode.ButtonEvent;
import org.firstinspires.ftc.teamcode.Motor8696;
import org.firstinspires.ftc.teamcode.MotorState;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.ServoTarget;

/**
 * Created by USER on 12/8/2017.
 */

@TeleOp(name="testNewBot", group="Temp")
public class NewRobot extends MecanumOpMode {

    /**
     * How many encoder counts the lifts need to
     * rotate to elevate the lift by one block.
     */
    private final static int LIFT_BLOCK_HEIGHT = 3000;

    private int liftPosition = 0;

    private int reverseLeft = 1;
    private int reverseRight = 1;

    @Override
    void buttonEvents() {
        super.buttonEvents();

        addButtonEvent(2, new ButtonEvent(Button.UP) {
            public void onDown() {
                liftPosition++;
                if (liftPosition > 3)
                    liftPosition = 3;
                setLiftTarget();
            }
        });

        addButtonEvent(2, new ButtonEvent(Button.DOWN) {
            public void onDown() {
                liftPosition--;
                if (liftPosition < -1)
                    liftPosition = -1;
                setLiftTarget();
            }
        });
    }

    private void setLiftTarget() {
        leftLift.setTargetPosition(-liftPosition * LIFT_BLOCK_HEIGHT);
        rightLift.setTargetPosition(-liftPosition * LIFT_BLOCK_HEIGHT);

        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        leftLift.setPower(0.3);
//        rightLift.setPower(0.3);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        buttonEvents();

        waitForStart();

        double power = 0.35;

        while (opModeIsActive()) {

            telemetry.addData("liftPos", liftPosition);

            if (collectorActive) {
                if (gamepad2.right_bumper) {
                    leftCollector.setPower(-power);
                    rightCollector.setPower(-power);
                } else if (gamepad2.b) {
                    leftCollector.setPower(power);
                    rightCollector.setPower(-power);
                } else if (gamepad2.x) {
                    leftCollector.setPower(-power);
                    rightCollector.setPower(power);
                } else {
                    leftCollector.setPower(power * reverseLeft);
                    rightCollector.setPower(power * reverseRight);
                }
            } else {
                leftCollector.setPower(0);
                rightCollector.setPower(0);
            }

            leftLift.setPower(gamepad2.left_stick_y);
            rightLift.setPower(gamepad2.left_stick_y);

            telemetry.addData("liftPower", gamepad2.left_stick_y);

            telemetry.addData("leftLift", leftLift.getCurrentPosition());
            telemetry.addData("rightLift", rightLift.getCurrentPosition());

            telemetry.addData("left target", leftLift.getTargetPosition());
            telemetry.addData("right target", rightLift.getTargetPosition());

            telemetry.addData("rightTrigger", "%.2f", 1-gamepad2.right_trigger);
            telemetry.addData("leftTrigger", "%.2f", gamepad2.left_trigger);

            ballPoosher.setPosition(gamepad2.left_trigger);

            telemetry.addData(leftCollector.getDeviceName(), robotState.motors[4].speed);
            telemetry.addData(rightCollector.getDeviceName(), robotState.motors[5].speed);

            if (!dpadDrive()) {
                if (reverse > 0)
                    verticalDrive(reverse * driveSpeed, gamepad1.left_stick_y, gamepad1.right_stick_y);
                else
                    verticalDrive(reverse * driveSpeed, gamepad1.right_stick_y, gamepad1.left_stick_y);
                horizontalDrive(reverse * driveSpeed, (gamepad1.right_trigger - gamepad1.left_trigger));

                for (Motor8696 motor : motors) {
                    motor.setMaxPower(Math.sqrt(2));
                }

                runMotors();
            }

            if (!cubePoosherTarget.runServo()) cubePoosherTarget = ServoTarget.NULL;

            runButtonEvents();
            periodic(100);

            telemetry.update();
        }
    }

    private void checkCollectors() {
        reverseLeft = reverseRight = 1;
        if (checkCollector(robotState.motors[4])) {
            reverseLeft = -1;
        }
        else if (checkCollector(robotState.motors[5])) {
            reverseRight = -1;
        }
    }

    private boolean checkCollector(MotorState motorState) {
        if (motorState.power == 0 || motorState.lastPower == 0)
            return false;
        if (motorState.speed < 1) {// TODO: change the shit out of this
            return true;
        }

        return false;
    }
}
