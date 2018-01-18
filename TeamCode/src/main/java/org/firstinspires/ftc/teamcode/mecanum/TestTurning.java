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
        initGyro();

        waitForStart();
        telemetry.log().clear();

        getGyroData();

        telemetry.log().add("%.2f", angles.firstAngle);
        autoTurn(45, 0.35, 10, true);
        telemetry.log().add("%.2f", angles.firstAngle);
        autoTurn(0, 0.35, 10, true);
        telemetry.log().add("%.2f", angles.firstAngle);
        autoTurn(-90, 0.35, 10, true);
        telemetry.log().add("%.2f", angles.firstAngle);
//
//        autoTurn(-180, 0.35, 10, true);
//
//        autoTurn(8010, 0.35, 10, true);
//
//        autoStrafe(10, 0.3, 10);

        sleep(10000);
        telemetry.update();

    }
}
