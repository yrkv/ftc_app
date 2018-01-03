package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by USER on 11/23/2017.
 */

public class MotorState {
    public int currentPosition, targetPosition;
    public double power, lastPower;

    public String deviceName;

    public double speed = 0;

    private DcMotor motor;

    private long lastUpdate;

    public MotorState(DcMotor motor) {
        currentPosition = motor.getCurrentPosition();
        targetPosition = motor.getTargetPosition();
        power = motor.getPower();

        deviceName = motor.getDeviceName();
        this.motor = motor;

        lastUpdate = System.currentTimeMillis();
    }

    public String toString() {
        return String.format("%s %d %d %.2f", deviceName, currentPosition, targetPosition, power);
    }

    public void update() {
        int newPosition = motor.getCurrentPosition();
        targetPosition = motor.getTargetPosition();
        lastPower = power;
        power = motor.getPower();

        int dPosition = newPosition - currentPosition;
        currentPosition = motor.getCurrentPosition();
        long dTime = System.currentTimeMillis() - lastUpdate;

        speed = (double) dPosition / dTime;

        lastUpdate = System.currentTimeMillis();
    }
}
