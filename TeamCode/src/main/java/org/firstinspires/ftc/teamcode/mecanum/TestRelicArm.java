package org.firstinspires.ftc.teamcode.mecanum;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.buttonEvents.Button;
import org.firstinspires.ftc.teamcode.buttonEvents.ButtonEvent;



@TeleOp(name = "relic arm test", group = "test")
public class TestRelicArm extends MecanumOpMode {

    protected CRServo relicExtend;
    protected Servo rotateRelic;
    protected Servo relicGrabber;

    @Override
    void buttonEvents() {
        addButtonEvent(1, new ButtonEvent(Button.A) { // TODO: adjust all controls
            public void onDown() {
                relicGrabber.setPosition(1);
            }
            public void onUp() {
                relicGrabber.setPosition(0);
            }
        });

        addButtonEvent(1, new ButtonEvent(Button.B) {
            public void onDown() {
                rotateRelic.setPosition(1);
            }
            public void onUp() {
                rotateRelic.setPosition(0);
            }
        });

        addButtonEvent(1, new ButtonEvent(Button.BACK) { // TODO: see if I can move it to dpad
            public void onDown() {
                relicExtend.setPower(1);
            }
            public void onUp() {
                relicExtend.setPower(0);
            }
        });

        addButtonEvent(1, new ButtonEvent(Button.Y) {
            public void onDown() {
                relicExtend.setPower(-1);
            }
            public void onUp() {
                relicExtend.setPower(0);
            }
        });
    }

    @Override
    public void runOpMode() throws InterruptedException {
//        initRobot();

        // TODO: integrate into main program
        relicExtend = hardwareMap.get(CRServo.class, "relicExtend");
        rotateRelic = hardwareMap.get(Servo.class, "rotateRelic");
        relicGrabber = hardwareMap.get(Servo.class, "relicGrabber");

        // TODO: scale ranges
        rotateRelic.scaleRange(0, 1);
        relicGrabber.scaleRange(0, 1);

        relicExtend.setPower(0);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad2.a)
                relicExtend.setPower(gamepad1.left_stick_y);
            if (gamepad2.b)
                rotateRelic.setPosition(gamepad1.right_stick_y);
            if (gamepad1.a)
                relicGrabber.setPosition(gamepad2.left_stick_y);

            telemetry.addData("extend", gamepad1.left_stick_y);
            telemetry.addData("rotate", gamepad1.right_stick_y);
            telemetry.addData("grabber", gamepad2.left_stick_y);

            telemetry.update();
        }
    }
}
