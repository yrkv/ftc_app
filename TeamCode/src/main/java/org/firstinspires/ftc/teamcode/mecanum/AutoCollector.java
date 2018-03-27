package org.firstinspires.ftc.teamcode.mecanum;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by USER on 1/23/2018.
 */

@Disabled
@Autonomous(name = "collector example", group = "testing")
public class AutoCollector extends MecanumOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        cubePoosher.setPosition(0);

        waitForStart();

        collectorActive = true;

        runtime.reset();

        while (opModeIsActive() && runtime.seconds() < 3) {
            runCollector();

            checkCollectors();

            periodic(100);

            telemetry.update();
        }

        sleep(2000);

        leftCollector.setPower(-0.2);
        rightCollector.setPower(-0.2);

        sleep(200);

        cubePoosher.setPosition(1);

        sleep(1000);
    }
}
