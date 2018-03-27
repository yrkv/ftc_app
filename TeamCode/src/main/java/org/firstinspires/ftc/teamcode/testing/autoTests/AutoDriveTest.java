package org.firstinspires.ftc.teamcode.testing.autoTests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.RunUntil;
import org.firstinspires.ftc.teamcode.mecanum.MecanumOpMode;

/**
 * Created by USER on 2/23/2018.
 */

@Disabled
@Autonomous(name = "test driving", group = "test")
public class AutoDriveTest extends MecanumOpMode {

    private double fastest = 0;
    private int testCaseNum = 0;

    public void runOpMode() throws InterruptedException {
        initRobot();
        initGyro();

        waitForStart();

        telemetry.log().clear();

//        testAutoDrive(10, 0.25, 10);
//        sleep(10000);
//        testAutoDrive(-10, -0.25, 10);
//        sleep(2000);
        testAutoStrafe(10, 0.25, 10);
        sleep(10000);
        testAutoStrafe(-10, -0.25, 10);
        sleep(2000);
//        testAutoDrive(40, 0.4, 10);
//        sleep(10000);
//        testAutoStrafe(-20, 0.5, 10);
//        sleep(2000);
    }

    private void testAutoDrive(double inches, double power, double timeoutSeconds) {
        testCaseNum++;

        getGyroData();
        fastest = 0;
        autoDrive(inches, power, timeoutSeconds, new RunUntil() {
            public boolean stop() {
                getGyroData();
                double currentSpeed = Math.sqrt(gravity.xAccel * gravity.xAccel
                        + gravity.yAccel * gravity.yAccel
                        + gravity.zAccel * gravity.zAccel);

                if (currentSpeed > fastest)
                    fastest = currentSpeed;
                return false;
            }
        });

        sleep(500);
        getGyroData();
        telemetry.log().add("%d drive orientation: %.2f", testCaseNum, angles.firstAngle);
        telemetry.log().add("%d drive fastest speed: %.2f %s", testCaseNum, fastest, gravity.unit.toString());
    }

    private void testAutoStrafe(double inches, double power, double timeoutSeconds) {
        testCaseNum++;

        getGyroData();
        fastest = 0;
        autoStrafe(inches, power, timeoutSeconds, new RunUntil() {
            public boolean stop() {
                getGyroData();
                double currentSpeed = Math.sqrt(gravity.xAccel * gravity.xAccel
                        + gravity.yAccel * gravity.yAccel
                        + gravity.zAccel * gravity.zAccel);

                if (currentSpeed > fastest)
                    fastest = currentSpeed;
                return false;
            }
        });

        sleep(500);
        getGyroData();
        telemetry.log().add("%d strafe orientation: %.2f", testCaseNum, angles.firstAngle);
        telemetry.log().add("%d strafe fastest speed: %.2f %s", testCaseNum, fastest, gravity.unit.toString());
    }
}
