package org.firstinspires.ftc.teamcode.mecanum;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.OpMode8696;

/**
 * Created by USER on 11/5/2017.
 */

public abstract class MecanumOpMode extends OpMode8696 {

    protected void initRobot() {
        super.initRobot();
        initMotors();

        leftBack  .setDirection(DcMotor.Direction.FORWARD);
        rightBack .setDirection(DcMotor.Direction.REVERSE);
        leftFront .setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

        rightBack .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack  .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        navx = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        gyro = navx;

        telemetry.log().add("Gyro Calibrating. Do Not Move!");

//         Wait until the gyro calibration is complete
        int loop = 0;
        while (navx.isCalibrating())  {
            loop++;
            telemetry.addData(">", loop);
            telemetry.update();
            sleep(50);
            idle();
        }
        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update();
    }
}
