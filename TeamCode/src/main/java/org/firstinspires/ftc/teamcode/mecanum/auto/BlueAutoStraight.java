package org.firstinspires.ftc.teamcode.mecanum.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="BlueStraight", group="Mecanum")
public class BlueAutoStraight extends BaseAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        setTeamColor(BLUE);

        super.runOpMode();
    }

    protected void alignToCryptobox() {
        autoDrive(24, 0.3, 8); // drive forward until robot is off of the platform.

        autoTurn(0, 0.4, 1, true);

        autoStrafe(5, 0.3, 2);
    }
}