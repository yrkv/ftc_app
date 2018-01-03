package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by USER on 11/23/2017.
 */

public class RobotState {
    public Acceleration gravity;
    public Orientation angles;

    /**
     * leftFront,
     * rightFront,
     * leftBack,
     * rightBack,
     * leftCollector,
     * rightCollector
     */
    public MotorState[] motors;

    public RobotState(OpMode8696 robot, DcMotor[] robotMotors) {
        motors = new MotorState[robotMotors.length];
        for (int i = 0; i < robotMotors.length; i++) {
            motors[i] = new MotorState(robotMotors[i]);
        }
        gravity = robot.gravity;
        angles = robot.angles;
    }

    public String toString() {
        String out = gravity.toString() + " ";
        out += angles;
        for (MotorState motor : motors) {
            out += "\n" + motor;
        }
        return out;
    }

    public void update(OpMode8696 robot) {
        gravity = robot.gravity;
        angles = robot.angles;

        for (MotorState motorState : motors) {
            motorState.update();
        }
    }
}
