package org.firstinspires.ftc.teamcode.mecanum;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.RunUntil;

/**
 * Created by USER on 1/9/2018.
 */

@Autonomous(name="RedTurn", group="Temp")
public class RedAutoTurn extends MecanumOpMode {
    private JewelDetector jewelDetector = null;

    private ColorSensor leftColor;
    private ColorSensor rightColor;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

//        leftColor = hardwareMap.get(ColorSensor.class, "leftColor"); // TODO: config
//        rightColor = hardwareMap.get(ColorSensor.class, "rightColor"); // TODO: config


        jewelDetector = new JewelDetector();
        jewelDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

        //Jewel Detector Settings
        jewelDetector.areaWeight = 0.02;
        jewelDetector.detectionMode = JewelDetector.JewelDetectionMode.MAX_AREA; // PERFECT_AREA
        //jewelDetector.perfectArea = 6500; <- Needed for PERFECT_AREA
        jewelDetector.debugContours = true;
        jewelDetector.maxDiffrence = 15;
        jewelDetector.ratioWeight = 15;
        jewelDetector.minArea = 700;

        jewelDetector.enable();

        waitForStart();

        JewelDetector.JewelOrder order = jewelDetector.getLastOrder();

        telemetry.log().add(order.name());

        switch (order) {
            case BLUE_RED:
                ballPoosher.setPosition(0.5); // TODO: set to 1 after shown that it works.
                sleep(200);
                autoTurn(-10, 0.3, 4, false);
                break;
            case RED_BLUE:
                ballPoosher.setPosition(0.5); // TODO: set to 1 after shown that it works.
                sleep(200);
                autoTurn(10, 0.3, 4, false);
                break;
            case UNKNOWN:
                telemetry.log().add("failed to detect jewels");
        }

        if (jewelDetector.getCameraView().isEnabled()) // paranoia
            jewelDetector.disable();
        else
            telemetry.log().add("cameraView not enabled when attempted to disable");

        ballPoosher.setPosition(0);
        sleep(100);

        autoTurn( 30, 0.3, 5, false);

        initVuforia();

        relicTrackables.activate();

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        telemetry.addData("vuMark", vuMark.name());
        telemetry.update();

        sleep(100000);

        autoDrive(20, 0.4, 8); // drive forward until robot is off of the platform.

        autoTurn(90, 0.4, 5, true);

        autoStrafe(80, 0.3, 10, new RunUntil() {
            @Override
            public boolean stop() {
                return rightColor.red() > 5;
            }
        });

        // for now, hardcoded relative to where it first hits the line
        // TODO: change it to where it can follow the line, add a directional autoDrive

        switch (vuMark) {
            case LEFT:
                autoStrafe(0, 0.4, 2);
                break;
            case CENTER:
                autoStrafe(6, 0.4, 3);
                break;
            case RIGHT:
                autoStrafe(12, 0.4, 4);
                break;
            default:
                autoStrafe(6, 0.4, 3);
        }

        autoDrive(10, 0.4, 5);

        leftCollector.setPower(-0.25);
        rightCollector.setPower(-0.25);
    }
}
