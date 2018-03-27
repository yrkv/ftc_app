package org.firstinspires.ftc.teamcode.mecanum.auto;

import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by USER on 2/8/2018.
 */

@Disabled
@Autonomous(name="Jewel Test", group="Test")
public class JewelTest extends BaseAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        initGyro();
        initJewelDetector();

        JewelDetector jewelDetector = getJewelDetector();

        telemetry.log().add("ready to start");

        waitForStart();

        while (opModeIsActive()) {
            ballPoosher.setPosition(gamepad1.left_stick_y);
            telemetry.addData("pos", gamepad1.left_stick_y);
            telemetry.update();
        }

        ballPoosher.setPosition(0);

        autoTurn(10, 0.4, 3, true);

        JewelDetector.JewelOrder order = jewelDetector.getLastOrder();

        if (jewelDetector.getCameraView().isEnabled()) // paranoia
            jewelDetector.disable();
        else
            telemetry.log().add("cameraView not enabled when attempted to disable");

        telemetry.log().add(order.name());

        autoTurn(0, 0.4, 3, true);

        pushBall(RED, order);
    }

    @Override
    protected void alignToCryptobox() {

    }
}
