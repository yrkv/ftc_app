package org.firstinspires.ftc.teamcode.mecanum;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by USER on 1/16/2018.
 */

@Disabled
@Autonomous(name = "Test IMU (my shit)", group = "Sensor")
public class TestIMU extends MecanumOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initGyro();
    }
}
