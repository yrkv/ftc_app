package org.firstinspires.ftc.teamcode.temp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by USER on 11/3/2017.
 */

@Autonomous(name="Full Auto BLUE", group ="Temp")
public class TempAutoBlueFull extends TempOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        initVuforia();
        relicTrackables.activate();

        waitForStart();

        cubeLinearArm.setPower(-0.5);

        sleep(400);

        cubeLinearArm.setPower(0);

//        pushBall(BLUE);

        autoTurn(15, 0.4, 3);

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        autoTurn(-90, 0.4, 5);
//        autoTurn(-90, 0.4, 2);

        double dist = 20;

        telemetry.log().add(vuMark.name());

        switch (vuMark) {
            case LEFT:
                autoDrive(dist, 0.5, 5);
                break;
            case CENTER:
                autoDrive(dist+6.5, 0.5, 5);
                break;
            case RIGHT:
                autoDrive(dist+13, 0.5, 5);
                break;
            default:
                autoDrive(dist+6.5, 0.5, 5);
                break;
        }

        autoTurn(-180, 0.5, 5);

        autoDrive(20, 0.4, 6);

        grabber1.setPosition(0.5);
        grabber2.setPosition(0.5);

        autoDrive(-5, 0.4, 3);
    }
}