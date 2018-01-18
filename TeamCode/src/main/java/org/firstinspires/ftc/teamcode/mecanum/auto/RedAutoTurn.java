package org.firstinspires.ftc.teamcode.mecanum.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="RedTurn", group="Mecanum")
public class RedAutoTurn extends BaseAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        setTeamColor(RED);

        super.runOpMode();
    }

    protected void alignToCryptobox() {
        autoDrive(-24, 0.4, 8); // drive forward until robot is off of the platform.

        autoTurn(90, 0.4, 8, true);
        sleep(100);
        autoTurn(90, 0.5, 1 , true);

//        autoStrafe(-5, 0.4, 2);
    }
}
