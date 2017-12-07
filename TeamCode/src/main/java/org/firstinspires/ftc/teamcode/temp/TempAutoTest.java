package org.firstinspires.ftc.teamcode.temp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by USER on 11/18/2017.
 */

@Autonomous(name="Temp Auto BONOBO", group ="Temp")
public class TempAutoTest extends TempOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        waitForStart();

//        tempAutoDrive(10, 0.5, 5); // works
//        tempAutoStrafe(10, 0.5, 5); // kinda works

//        autoTurn();

        grabber1.setPosition(1);
        grabber2.setPosition(1);

        sleep(400);

        cubeLinearArm.setPower(-0.5);

        sleep(400);

        cubeLinearArm.setPower(0);



        autoDrive(50, 0.5, 10);

        grabber1.setPosition(0);
        grabber2.setPosition(0);

        autoDrive(-4, 0.5, 2);
    }
}
