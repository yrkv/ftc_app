package org.firstinspires.ftc.teamcode.temp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Button;
import org.firstinspires.ftc.teamcode.ButtonEvent;
import org.firstinspires.ftc.teamcode.Motor8696;

/**
 * Created by USER on 10/13/2017.
 */

@TeleOp(name="tempDriveDifferent", group="Temp")
public class TempDriveDifferentDriving extends TempOpMode {

    private int reverse = 1;
    private static final double DPAD_MULT = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        buttonEvents();

//        initVuforia();

        waitForStart();

        while (opModeIsActive()) {
            if (!dpadDrive()) {
                for (Motor8696 motor : motors) {
                    motor.setMaxPower(Math.sqrt(2));
                }

//                if (getMagnitude(gamepad1.left_stick_x, gamepad1.left_stick_y) >= 0.25) {
                    verticalDrive(reverse * 0.5, gamepad1.left_stick_y, gamepad1.left_stick_y);
                    horizontalDrive(reverse, gamepad1.left_stick_x);
//                }

                double ry = gamepad1.right_stick_y;
                double rx = -gamepad1.right_stick_x;

                double rightStickAngle = Math.atan(ry / rx) - Math.PI / 2;
                if (rx > 0) rightStickAngle += Math.PI;

                if (!Double.isNaN(rightStickAngle) && getMagnitude(rx, ry) >= 0.5) {
                    telemetry.addData("angle", "%.2f", rightStickAngle * 180 / Math.PI);
                    onHeading(rightStickAngle * 180 / Math.PI, 0.8, 1, false);
                }

                runMotors();
            }


            double armPower = gamepad2.dpad_up ?  -0.5 : (gamepad2.dpad_down ?  0.5 : 0);


            for (Motor8696 motor : motors) {
                telemetry.addData(motor.getDeviceName(), motor.getCurrentPosition());
            }

            cubeLinearArm.setPower(armPower);

            runButtonEvents();
//            periodic(100);
            telemetry.update();
        }
    }

    protected void periodic() {

    }

//    private boolean verifyDrive() {
//
//    }
}
