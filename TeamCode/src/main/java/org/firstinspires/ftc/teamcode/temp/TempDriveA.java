package org.firstinspires.ftc.teamcode.temp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Button;
import org.firstinspires.ftc.teamcode.ButtonEvent;
import org.firstinspires.ftc.teamcode.Motor8696;

/**
 * Created by USER on 10/13/2017.
 */

@TeleOp(name="tempDriveA", group="Temp")
public class TempDriveA extends TempOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        buttonEvents();

//        initVuforia();

        waitForStart();

        while (opModeIsActive()) {
            if (!dpadDrive()) {
                verticalDrive(reverse * driveSpeed, gamepad1.left_stick_y, gamepad1.right_stick_y);
                horizontalDrive(reverse, (gamepad1.right_stick_x + gamepad1.left_stick_x)/2);

                for (Motor8696 motor : motors) {
                    motor.setMaxPower(Math.sqrt(2));
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
