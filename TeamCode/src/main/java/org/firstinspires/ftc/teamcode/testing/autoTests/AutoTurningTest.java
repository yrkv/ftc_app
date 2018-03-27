package org.firstinspires.ftc.teamcode.testing.autoTests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.mecanum.MecanumOpMode;

/**
 * Created by USER on 1/7/2018.
 */

@Disabled
@Autonomous(name="test turning", group="test")
public class AutoTurningTest extends MecanumOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        enableEncoders(); // probably pointless // definitely pointless
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
        autoTurn(0, 0.3, 10, true);
        telemetry.log().add("%.2f", angles.firstAngle);
        autoTurn(90, 0.3, 4, false);
        telemetry.log().add("%.2f false", angles.firstAngle);
        autoTurn(80, 0.1, 4, true);
        telemetry.log().add("%.2f", angles.firstAngle);
        autoTurn(70, 0.1, 4, false);
        telemetry.log().add("%.2f false", angles.firstAngle);


        sleep(10000);
        telemetry.update();

    }
}
