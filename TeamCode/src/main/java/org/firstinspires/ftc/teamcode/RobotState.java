package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by USER on 11/23/2017.
 */

public class RobotState {
    public Acceleration gravity;
    public Orientation angles;

    public MotorState[] motors;

    public RobotState(OpMode8696 robot) {
        for (int i = 0; i < robot.motors.length; i++) {
            motors[i] = new MotorState(robot.motors[i]);
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
}
