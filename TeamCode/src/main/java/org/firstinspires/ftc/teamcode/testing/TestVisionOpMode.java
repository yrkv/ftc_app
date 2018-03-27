package org.firstinspires.ftc.teamcode.testing;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by guinea on 10/5/17.
 * This is a sample opmode that demonstrates the use of an OpenCVPipeline with FTC code.
 * When the x button is pressed on controller one, the camera is set to show areas of the image
 * where a certain color is, in this case, blue.
 *
 */
@Disabled
@TeleOp(name="Blue Vision")
public class TestVisionOpMode extends OpMode {
    TestVision blueVision;
    @Override
    public void init() {
        blueVision = new TestVision();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        blueVision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        blueVision.setShowBlue(false);
        // start the vision system
        blueVision.enable();
    }

    @Override
    public void loop() {
        blueVision.setShowBlue(gamepad1.x);
    }

    public void stop() {
        // stop the vision system
        blueVision.disable();
    }
}