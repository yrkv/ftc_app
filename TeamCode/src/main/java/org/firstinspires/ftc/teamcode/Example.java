package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by USER on 10/27/2017.
 */

@Disabled
@TeleOp(name="tempDrive", group="Linear Opmode")
public class Example extends OpMode8696 {
    DcMotor motor;
    Servo servo;

    ColorSensor colorSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotor.class, "motor");
        servo = hardwareMap.get(Servo.class, "servo");

        colorSensor = hardwareMap.get(ColorSensor.class, "color");

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servo.setPosition(0.5);

        waitForStart();

//        int currentPos = motor.getCurrentPosition();
//        int target = currentPos + 400 * 10;
//
//        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        motor.setTargetPosition(target);
//
//        motor.setPower(0.5);
//
//        while (motor.isBusy()) {
//            idle();
//        }
//
//        motor.setPower(0);
//
//        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        while (opModeIsActive()) {

            telemetry.addData(">", colorSensor.red());
            servo.setPosition(gamepad1.left_stick_y / 2 + 1);
            telemetry.update();
        }

    }
}
