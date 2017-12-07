package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Motor8696;
import org.firstinspires.ftc.teamcode.OpMode8696;

/**
 * Created by USER on 11/17/2017.
 */

@Disabled
@TeleOp(name="testCollector", group="Temp")
public class testCollector extends OpMode8696 {
    Motor8696 left;
    Motor8696 right;

    @Override
    public void runOpMode() throws InterruptedException {
        left = new Motor8696(hardwareMap.get(DcMotor.class, "left"));
        right = new Motor8696(hardwareMap.get(DcMotor.class, "right"));

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            left.setPower(-gamepad1.left_stick_y);
            right.setPower(gamepad1.right_stick_y);
        }
    }
}
