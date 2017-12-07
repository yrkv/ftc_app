package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by USER on 11/23/2017.
 */

public class MotorState {
    public int currentPosition, targetPosition;
    public double power;

    public String deviceName;

    public MotorState(Motor8696 motor) {
        currentPosition = motor.getCurrentPosition();
        targetPosition = motor.getTargetPosition();
        power = motor.getPower();

        deviceName = motor.getDeviceName();
    }

    public String toString() {
        return String.format("%s %d %d %2.f", deviceName, currentPosition, targetPosition, power);
    }
}
