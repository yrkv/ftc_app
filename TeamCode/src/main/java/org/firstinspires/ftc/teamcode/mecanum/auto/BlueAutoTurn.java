package org.firstinspires.ftc.teamcode.mecanum.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name="BlueTurn", group="Mecanum")
public class BlueAutoTurn extends BaseAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        setTeamColor(RED);

        super.runOpMode();
    }

    protected void alignToCryptobox() {
        autoDrive(24, 0.4, 8); // drive forward until robot is off of the platform.

        autoTurn(90, 0.4, 8, true);
        sleep(100);
        autoTurn(90, 0.5, 1 , true);

//        autoStrafe(-5, 0.4, 2);
    }

    protected void pickCryptoboxColumn(RelicRecoveryVuMark vuMark) {
        // for now, hardcoded
        // TODO: change it to where it can follow the line, add a directional autoDrive

        switch (vuMark) {
            case LEFT:
                autoStrafe(0, 0.4, 2);
                break;
            case CENTER:
                autoStrafe(7.5, 0.4, 3);
                break;
            case RIGHT:
                autoStrafe(15, 0.4, 4);
                break;
            default:
                autoStrafe(7.5, 0.4, 3);
        }
    }
}
