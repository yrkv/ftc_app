package org.firstinspires.ftc.teamcode.mecanum.auto;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.mecanum.MecanumOpMode;

/**
 * Created by USER on 1/9/2018.
 */

public abstract class BaseAuto extends MecanumOpMode {
    private JewelDetector jewelDetector = null;

    private ColorSensor leftColor;
    private ColorSensor rightColor;

    private boolean teamColor;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        initGyro();

//        leftColor = hardwareMap.get(ColorSensor.class, "leftColor"); // TODO: config
//        rightColor = hardwareMap.get(ColorSensor.class, "rightColor"); // TODO: config

        initJewelDetector();

        sleep(1000);

        telemetry.log().clear();

        JewelDetector.JewelOrder order = jewelDetector.getLastOrder();

        if (jewelDetector.getCameraView().isEnabled()) // paranoia
            jewelDetector.disable();
        else
            telemetry.log().add("cameraView not enabled when attempted to disable");

        initVuforia();

        relicTrackables.activate();

        telemetry.log().add(order.name());
        waitForStart();

        pushBall(teamColor, order);

        ballPoosher.setPosition(0);
        sleep(1000);

        autoTurn( 25, 0.3, 5, true); // TODO: find real value

        sleep(2000);

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        telemetry.addData("vuMark", vuMark.name());
        telemetry.update();

        autoTurn( 0, 0.3, 5, true);
        sleep(100);
        autoTurn( 0, 0.3, 1, true);

        alignToCryptobox();

        pickCryptoboxColumn(vuMark);

        sleep(1000);

        placeGlyph();
    }

    protected void setTeamColor(boolean teamColor) {
        this.teamColor = teamColor;
    }

    protected void initJewelDetector() {
        telemetry.log().add("initializing jewel detector...");
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

        telemetry.log().add("jewel detector initialized");
    }

    protected void pushBall(boolean teamColor, JewelDetector.JewelOrder order) {
        int reverse = teamColor ? 1 : -1;
        switch (order) {
            case BLUE_RED:
                ballPoosher.setPosition(0.5); // TODO: set to 1
                sleep(200);
                autoTurn(10 * reverse, 0.3, 4, false);
                break;
            case RED_BLUE:
                ballPoosher.setPosition(0.5); // TODO: set to 1
                sleep(200);
                autoTurn(-10 * reverse, 0.3, 4, false);
                break;
            case UNKNOWN:
                telemetry.log().add("failed to detect jewels");
        }
    }

    protected abstract void alignToCryptobox();

    protected void pickCryptoboxColumn(RelicRecoveryVuMark vuMark) {
        // for now, hardcoded
        // TODO: change it to where it can follow the line, add a directional autoDrive

        switch (vuMark) {
            case LEFT:
                autoStrafe(-15, 0.4, 2);
                break;
            case CENTER:
                autoStrafe(-7.5, 0.4, 3);
                break;
            case RIGHT:
                autoStrafe(0, 0.4, 4);
                break;
            default:
                autoStrafe(-7.5, 0.4, 3);
        }
    }

    protected void placeGlyph() {
        autoDrive(8, 0.15, 5);

        leftCollector.setPower(-0.3);
        rightCollector.setPower(-0.3);

        sleep(1000);

        autoDrive(-5, 0.2, 3);


        sleep(1000);

        leftCollector.setPower(0);
        rightCollector.setPower(0);
    }
}
