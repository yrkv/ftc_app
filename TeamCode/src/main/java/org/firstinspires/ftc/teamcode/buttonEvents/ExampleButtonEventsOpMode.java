package org.firstinspires.ftc.teamcode.buttonEvents;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp(name = "example button events", group = "test")
public class ExampleButtonEventsOpMode extends LinearOpMode {

    /**
     * Object that manages all the button events. Must be initialized
     * during runOpMode, as the gamepads don't exist until then.
     */
    private ButtonEventManager buttonEvents;

    // not a real motor. just used to show how it could be used.
    private DcMotor someMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        // Give the button manager references to both gamepads.
        // These are the ones in the code, so it works even if they are
        // unplugged and plugged back in.
        buttonEvents = new ButtonEventManager(gamepad1, gamepad2);

        someMotor = hardwareMap.get(DcMotor.class, "This is not a real motor. I just needed a hardware object so that I could show how my button events system works.");


        // add a new event on gamepad 1 when A is pressed.
        buttonEvents.addButtonEvent(1, new ButtonEvent(Button.A) {
            public void onDown() { // when it's initially pressed down
                someMotor.setPower(1);
            }
            public void onUp() { // when it's released
                someMotor.setPower(0);
            }

            // there are also whileUp() and whileDown(), but those are rarely used.
        });


        waitForStart();


        while (opModeIsActive()) {
            // do stuff
            // idk what your program would do

            // but call this somewhere
            buttonEvents.run();;
        }
    }
}
