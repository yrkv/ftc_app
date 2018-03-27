package org.firstinspires.ftc.teamcode.mecanum;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by USER on 1/23/2018.
 */

@Disabled
@Autonomous(name = "fix collector", group = "testing")
public class FixCollector extends MecanumOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        waitForStart();

        collectorActive = true;

        while (opModeIsActive()) {
            runCollector();

            checkCollectors();

            periodic(100);

            telemetry.update();
        }
    }
}
