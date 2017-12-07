package org.firstinspires.ftc.teamcode.temp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by USER on 11/25/2017.
 */

@Autonomous(name="Temp Auto TURNING", group ="Temp")
public class TempAutoTurn extends TempOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 50);

        autoTurn(90, 0.4, 10);
    }
}
