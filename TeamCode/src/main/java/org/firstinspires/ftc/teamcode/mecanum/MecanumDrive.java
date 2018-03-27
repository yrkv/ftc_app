package org.firstinspires.ftc.teamcode.mecanum;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by USER on 10/13/2017.
 */

@Disabled
@TeleOp(name="mecanumDrive", group="Linear Opmode")
public class MecanumDrive extends MecanumOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
//        initVuforia();

        waitForStart();
        telemetry.log().clear();

        while (opModeIsActive()) {
            if (gamepad1.a)
                driveForward(0.5);
            else
                mecanumTeleOpDrive();

            buttonEvents.run();

            telemetry.update();

            idle();
        }
    }

    private void driveForward(double mult) {
        leftFront.setPower(mult);
        leftBack.setPower(mult);
        rightFront.setPower(mult);
        rightBack.setPower(mult);
    }
}
