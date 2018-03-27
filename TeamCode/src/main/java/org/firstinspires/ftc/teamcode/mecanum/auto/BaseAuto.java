package org.firstinspires.ftc.teamcode.mecanum.auto;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.RunUntil;
import org.firstinspires.ftc.teamcode.mecanum.MecanumOpMode;

/**
 * Base autonomous class that removes the correct jewel, places a glyph, and parks in the safe zone.
 * The {@link com.qualcomm.robotcore.eventloop.opmode.Autonomous} opModes all extend this class.
 * They declare their color and each autonomous has a different method to align to the cryptobox.
 */

public abstract class BaseAuto extends MecanumOpMode {
    private JewelDetector jewelDetector = null;

    private ColorSensor leftColor;
    private ColorSensor rightColor;

    protected RelicRecoveryVuMark foundMark = RelicRecoveryVuMark.UNKNOWN;

    private boolean teamColor;

    protected RunUntil findVuMark = new RunUntil() {
        public boolean stop() {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                foundMark = vuMark;
                return true;
            }

            return false;
        }
    };



    /**
     * Main autonomous code. <br/>
     *      1. initialize robot, gyro, jewel detector <br/>
     *      2. read jewel order after waiting 1 second <br/>
     *      3. wait for start <br/>
     *      4. initialize Vuforia <br/>
     *      5. remove the correct jewel <br/>
     *      6. turn and scan pictograph <br/>
     *      7. align to starting rotation <br/>
     *      8. drive/align to cryptobox <br/>
     *      9. move to correct column <br/>
     *      10.place the glyph
     * @throws InterruptedException When the program is stopped.
     */
    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        initGyro();

//        leftColor = hardwareMap.get(ColorSensor.class, "leftColor"); // TODO: add sensors and program them
//        rightColor = hardwareMap.get(ColorSensor.class, "rightColor");

        initJewelDetector();

        sleep(1000);

        telemetry.log().add(jewelDetector.getLastOrder().name());
        telemetry.log().add("ready to start");

        waitForStart();

        autoTurn(5, 0.25, 3, true);

        JewelDetector.JewelOrder order = jewelDetector.getLastOrder();

        if (jewelDetector.getCameraView().isEnabled()) // paranoia
            jewelDetector.disable();
        else
            telemetry.log().add("cameraView not enabled when attempted to disable");

        telemetry.log().add(order.name());

        autoTurn(0, 0.2, 3, true);

        initVuforia();
        relicTrackables.activate();

        pushBall(teamColor, order);

        ballPoosher.setPosition(0);
        sleep(500);

        turnFindVuMark();

        sleep(1000);

        RelicRecoveryVuMark vuMark;

        if (foundMark == RelicRecoveryVuMark.UNKNOWN)
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
        else
            vuMark = foundMark;


        telemetry.addData("vuMark", vuMark.name());
        telemetry.update();

        autoTurn( 0, 0.2, 5, true);
        sleep(100);
        autoTurn( 0, 0.25, 1, true);

        alignToCryptobox();

        pickCryptoboxColumn(teamColor, vuMark);

        sleep(1000);

        placeGlyph(); // TODO: make it use collectors when placing the cube, see how that affects it.
    }

    /**
     * Tells the program which color jewel to go for and adjusts a few other parts of it.
     * @param teamColor the color of the autonomous
     * @see #pushBall(boolean, JewelDetector.JewelOrder)
     * @see #pickCryptoboxColumn(boolean, RelicRecoveryVuMark)
     */
    protected void setTeamColor(boolean teamColor) {
        this.teamColor = teamColor;
    }

    /**
     * Initialize DogeCV for detecting jewel orientation
     */
    protected void initJewelDetector() {
//        telemetry.log().clear();
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

    /**
     * Remove the correct jewel during autonomous by moving a servo down and turning the robot
     * @param teamColor The color of the autonomous this is being used by
     * @param order jewel order returned by DogeCV
     */
    protected void pushBall(boolean teamColor, JewelDetector.JewelOrder order) {
        int reverse = teamColor ? 1 : -1;
        switch (order) {
            case BLUE_RED:
                ballPoosher.setPosition(1);
                sleep(200);
                autoTurn(10 * reverse, 0.3, 4, false);
                break;
            case RED_BLUE:
                ballPoosher.setPosition(1);
                sleep(200);
                autoTurn(-10 * reverse, 0.3, 4, false);
                break;
            case UNKNOWN:
                telemetry.log().add("failed to detect jewels");
        }
    }

    /**
     * Turn the robot so that it can see the VuMark.
     */
    protected void turnFindVuMark() {
        autoTurn( 30, 0.2, 5, true);
    }

    /**
     * Move the robot to a set position relative to the cryptobox.
     * This is the main part that distinguishes the various
     * autonomous programs.
     */
    protected abstract void alignToCryptobox();

    /**
     * Use the previously scanned pictograph to strafe to the correct column in the cryptobox
     * @param vuMark the previously scanned pictograph
     */
    protected void pickCryptoboxColumn(boolean teamColor, RelicRecoveryVuMark vuMark) {
        // for now, hardcoded
        // TODO: change it to where it can follow the line, add a directional autoDrive

        double add = teamColor ? 0 : 14;

        switch (vuMark) {
            case LEFT:
                autoStrafe(-14 + add, 0.4, 2);
                break;
            case CENTER:
                autoStrafe(-7 + add, 0.4, 3);
                break;
            case RIGHT:
                autoStrafe(0 + add, 0.4, 4);
                break;
            default:
                autoStrafe(-7 + add, 0.4, 3);
        }
    }

    /**
     * Uses the collector motors to eject the glyph into a cryptobox column.
     * This method can place the glyphs into the cryptobox even if it isn't
     * aligned properly. Whenever it is misaligned, whichever side of the
     * cryptobox divider has more distance has more torque, so the glyph is
     * pushed even farther in that direction, magnifying the effect.
     */
    protected void placeGlyph() {
        autoDrive(12, 0.2, 5);

        leftCollector.setPower(-0.3);
        rightCollector.setPower(-0.3);

        sleep(1000);

        autoDrive(-5, 0.2, 3);


        sleep(1000);

        leftCollector.setPower(0);
        rightCollector.setPower(0);
    }

    @Override
    public void stop() {
        super.stop();
        if (jewelDetector.getCameraView().isEnabled()) // paranoia
            jewelDetector.disable();
    }

    public JewelDetector getJewelDetector() {
        return jewelDetector;
    }
}
