package org.firstinspires.ftc.teamcode.mecanum;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by USER on 1/7/2018.
 */

@TeleOp(name="testNewAuto", group="Temp")
public class TestTurning extends MecanumOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        waitForStart();

        autoTurn(45, 0.35, 10, true);

        autoTurn(0, 0.35, 10, true);

        autoTurn(-90, 0.35, 10, true);
//
//        autoTurn(-180, 0.35, 10, true);
//
//        autoTurn(8010, 0.35, 10, true);
//
//        autoStrafe(10, 0.3, 10);

        getGyroData();
        telemetry.addData("angle", angles.firstAngle);
        telemetry.update();

    }
}
