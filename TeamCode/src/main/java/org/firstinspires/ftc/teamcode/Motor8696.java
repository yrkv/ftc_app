package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.configuration.MotorConfigurationType;

/**
 * Wrapper class that allows me to edit the standard DcMotor methods and add my own functionality.
 */

// this class is confusing and hard to work with. It should
// probably be removed and everything else refactored to not use it.
public class Motor8696 implements DcMotor {
    private DcMotor motor;

    public static final int COUNTS_PER_REVOLUTION = (int) (1440 * 10 / 26.5);

    private double maxPower = 1;
    private double currentPower = 0;

    private double scalePower = 1;

    private int prevCounts = 0;
    private boolean resetEncoder = true;

    private int lastPosition = 0;

    public DcMotor getMotor() {
        return motor;
    }

    public Motor8696(DcMotor motor) {
        this.motor = motor;
    }

    public void setMaxPower(double power) {
        maxPower = power;
    }

    public void setScalePower(double power) {
        scalePower = power;
    }

    public void addPower(double power, double max) {
        currentPower += power;
        maxPower += max;
    }

    public void setRelativeTarget(int counts) {
        setTargetPosition(prevCounts + counts);
    }

    public void storePosition() {
        prevCounts = getCurrentPosition();
    }

    public void saveCurrentPosition() {
        lastPosition = getCurrentPosition();
    }

    public void addPower(double power) {
        currentPower += power;
    }

    public void setPower() {
        double power = currentPower / maxPower;
        power = (power > 1) ? 1 : ((power < -1) ? -1 : power);
        if (Double.isNaN(power)) power = 0;
        setPower(power);

        reset(); // reset everything after done using it in an iteration.
    }

    public void reset() {
        currentPower = 0;
        maxPower = 1;
    }

    public double getCurrentPower() {
        return currentPower;
    }

    public double getMaxPower() {
        return maxPower;
    }

    public static boolean motorsBusy(Motor8696[] motors) {
        for (Motor8696 motor : motors) {
            if (!motor.isBusy())
                return false;
        }
        return true;
    }

    @Override
    public MotorConfigurationType getMotorType() {
        return motor.getMotorType();
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        motor.setMotorType(motorType);
    }

    @Override
    public DcMotorController getController() {
        return motor.getController();
    }

    @Override
    public int getPortNumber() {
        return motor.getPortNumber();
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return motor.getZeroPowerBehavior();
    }

    @Override
    public void setPowerFloat() {
        motor.setPowerFloat();
    }

    @Override
    public boolean getPowerFloat() {
        return motor.getPowerFloat();
    }

    @Override
    public void setTargetPosition(int position) {
        motor.setTargetPosition(position);
    }

    @Override
    public int getTargetPosition() {
        return motor.getTargetPosition();
    }

    @Override
    public boolean isBusy() {
        return motor.isBusy();
    }

    @Override
    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    @Override
    public void setMode(RunMode mode) {
        motor.setMode(mode);
    }

    @Override
    public RunMode getMode() {
        return motor.getMode();
    }

    @Override
    public void setDirection(Direction direction) {
        motor.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return motor.getDirection();
    }

    @Override
    public void setPower(double power) {
        power = power * scalePower;
        power = (power > 1) ? 1 : ((power < -1) ? -1 : power);
        currentPower = power;
        motor.setPower(power);
    }

    @Override
    public double getPower() {
        return motor.getPower();
    }

    @Override
    public Manufacturer getManufacturer() {
        return motor.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return motor.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return motor.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return motor.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        motor.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        motor.close();
    }
}
