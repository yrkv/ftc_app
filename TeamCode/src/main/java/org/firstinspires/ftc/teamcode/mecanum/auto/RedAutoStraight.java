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
        autoDrive(-20, 0.4, 8); // drive forward until robot is off of the platform.

        autoTurn(180, 0.4, 8, true);
        sleep(100);
        autoTurn(180, 0.5, 1 , true);

        autoStrafe(-5, 0.4, 2);
    }
}