package org.firstinspires.ftc.teamcode.mecanum.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="RedStraight", group="Mecanum")
public class RedAutoStraight extends BaseAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        setTeamColor(RED);

        super.runOpMode();
    }

    protected void alignToCryptobox() {
        autoDrive(-24, 0.3, 8); // drive forward until robot is off of the platform.

        autoTurn(180, 0.3, 8, true);
        sleep(100);
        autoTurn(180, 0.4, 1 , true);

        autoStrafe(-5, 0.3, 5);
    }
}