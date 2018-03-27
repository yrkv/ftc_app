package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.buttonEvents.ButtonEvent;
import org.firstinspires.ftc.teamcode.buttonEvents.ButtonEventManager;

// TODO: figure out how to use adb to graph some numbers.
// TODO: use it to optimize turning and driving in auto
// TODO: testing framework

/**
 * Superclass used by all of team 8696's opModes.
 * Contains all the methods and functionality that
 * any generic robot might have.
 */
public abstract class OpMode8696 extends LinearOpMode8696 {

    protected Motor8696 leftBack;
    protected Motor8696 rightBack;
    protected Motor8696 leftFront;
    protected Motor8696 rightFront;

    protected ElapsedTime runtime = new ElapsedTime();

    /**
     * Array containing the four motors in the drive train <br>
     * leftFront, rightFront, leftBack, rightBack
     *
     * @see Motor8696
     */
    protected Motor8696[] motors = new Motor8696[4];

    protected VuforiaLocalizer vuforia;
    protected VuforiaTrackables relicTrackables;

    protected BNO055IMU imu;

    protected Acceleration gravity;
    protected Orientation angles;

    protected double driveSpeed = 1;

    public static final boolean RED = true;
    public static final boolean BLUE = false;

    /**
     * constant for how many encoder counts are
     * equivalent to rotating the robot one degree.
     */
    private static final double TURN_COEFFICIENT = 15 / 1.4;

    private long lastPeriodicCall = 0;
    protected VuforiaTrackable relicTemplate;

    protected ButtonEventManager buttonEvents;

    /**
     * Check if enough time has passed to call {@link #periodic()} again.
     * If so, call it. Should be called every iteration of the main loop.
     * @param ms number of milliseconds between every {@link #periodic()}
     * @see #periodic()
     */
    protected void periodic(long ms) {
        long t = System.currentTimeMillis();
        if (t >= lastPeriodicCall + ms) {
            lastPeriodicCall = t;
            periodic();
        }
    }

    /**
     * Method that will be called periodically.
     * @see #periodic(long ms)
     */
    protected void periodic() {

    }

    protected void initRobot() {
        initDriveTrain();
        lastPeriodicCall = System.currentTimeMillis();
        buttonEvents = new ButtonEventManager(gamepad1, gamepad2);
    }

    protected void initDriveTrain() {
        leftFront  = new Motor8696(hardwareMap.get(DcMotor.class, "leftFront")); //0
        rightFront = new Motor8696(hardwareMap.get(DcMotor.class, "rightFront"));//1
        leftBack   = new Motor8696(hardwareMap.get(DcMotor.class, "leftBack"));  //2
        rightBack  = new Motor8696(hardwareMap.get(DcMotor.class, "rightBack")); //3

        motors[0] = leftBack;
        motors[1] = rightBack;
        motors[2] = leftFront;
        motors[3] = rightFront;
    }

    protected void extendServo(Servo servo, double min, double max) {
        // Confirm its an extended range servo controller before we try to set to avoid crash
        if (servo.getController() instanceof ServoControllerEx) {
            ServoControllerEx controller = (ServoControllerEx) servo.getController();
            int port = servo.getPortNumber();
            PwmControl.PwmRange range = new PwmControl.PwmRange(min, max); // 553, 2425
            controller.setServoPwmRange(port, range);
        }
    }

    protected void initVuforia() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AXsBBW7/////AAAAGXj7SVf450terrL5QOqPUr1Tozrj/sG57Z/tukLNECvwVhLUNaNxKv783tA6U2Kze0Hs+9EpVCJ8PzhKRCocFqWqDdZbqjktD2McMriGHUtCiIfoFyF5xKCZ11QBmMNTRBRkqV/s0HWgxkD41BA8d3ZlfS9zF7Vgh1397O35rqCY8KyjTqtaPzbxecZWb96/Bpq0Ct9u/e0e35d0+Vth/VdGp3vLMRFPNzPEZlJ6/VDQlgeHobmzJ7ccHKb6k7WPUC7vDyZEZXyIQPnAJoLbHT+j4kYFnVuFUaok5jrNn8TknXxpgRSvQTsxeilOQQtSxn/a9SNiR7pnpqRjLWAe0E1H5qu3a952fwo7PlGxkzk1";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        this.vuforia = new ClosableVuforiaLocalizer(parameters);

        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
    }

    /**
     * Add events for a specific button
     * @param gamepad which gamepad the button is on
     * @param event the event to be called
     */
    protected void addButtonEvent(int gamepad, ButtonEvent event) {
        buttonEvents.addButtonEvent(gamepad, event);
    }

    protected double getGamepadAngle(double x, double y) {
        double angle  = Math.atan(-y / x);
        if (x < 0) angle += Math.PI;

        return angle;
    }

    protected double getMagnitude(double x, double y) {
        double magnitude = Math.sqrt(x*x + y*y);
        return Range.clip(magnitude, -1, 1);
    }

    protected void autoTurn(double angle, double power, double timeoutSeconds, boolean useEncoders) {
        autoTurn(angle, power, timeoutSeconds, useEncoders, new RunUntil() {
            public boolean stop() {
                return false;
            }
        });
    }

    protected void autoTurn(double angle, double power, double timeoutSeconds, boolean useEncoders, RunUntil runUntil) {
        runtime.reset();
        getGyroData();

        while (opModeIsActive() &&
                onHeading(angle, power, 1.0, useEncoders) &&
                runtime.seconds() < timeoutSeconds &&
                !runUntil.stop()) {
            idle();
            runMotors();
            for (Motor8696 motor: motors) {
                motor.reset();
            }
        }
        runMotors();
    }

    protected boolean onHeading(double angle, double power, double maxError, boolean useEncoders) {
        getGyroData();

        double diff = adjustAngle(angle, angles.firstAngle);

        if (Math.abs(diff) > maxError) {
            if (useEncoders)
                encoderTurning(diff, power);
            else
                crappyTurning(diff, power);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Incrementally rotate the robot using the encoders, which makes use of the built in PID stuff.
     *
     * @param diff how much the robot needs to rotate to get to its target orientation.
     * @param power how much power to send to the motors.
     */
    private void encoderTurning(double diff, double power) {
        for (Motor8696 motor : motors) {
            motor.storePosition();
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        leftBack  .setRelativeTarget((int) (diff * TURN_COEFFICIENT));
        rightBack .setRelativeTarget((int)-(diff * TURN_COEFFICIENT));
        leftFront .setRelativeTarget((int) (diff * TURN_COEFFICIENT));
        rightFront.setRelativeTarget((int)-(diff * TURN_COEFFICIENT));

        for (Motor8696 motor : motors) {
            motor.addPower(power);
        }
    }

    /**
     * Incrementally rotate the robot using just the powers. Adjust the power as it gets close to the target.
     *
     * @param diff how much the robot needs to rotate to get to its target orientation.
     * @param power how much power to send to the motors.
     */
    private void crappyTurning(double diff, double power) {
        for (Motor8696 motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData(motor.getPortNumber() + "", motor.getCurrentPosition());
        }

        double temp = diff/30 * power;
        if (Math.abs(temp) < Math.abs(power))
            power = temp;
        else if (temp < 0) {
            power *= -1;
        }
        if (Math.abs(power) <= 0.15) {
            if (power < 0)
                power = -0.15;
            else
                power = 0.15;
        }

        telemetry.addData("power", power);
        telemetry.update();

        leftBack  .addPower( power);
        rightBack .addPower(-power);
        leftFront .addPower( power);
        rightFront.addPower(-power);
    }

    /**
     * Calculate the difference between two angles and set the output to be in a range.
     * Used for calculating how far the robot needs to turn to get to a target rotation.
     * @param target The target rotation
     * @param currentRotation The current robot orientation, (preferably) found by a gyro sensor.
     * @return Adjusted angle, -180 <= angle <= 180
     */
    protected double adjustAngle(double target, double currentRotation) {
        double diff = target - currentRotation;
        while (Math.abs(diff) > 180) {
            target += (diff >= 180) ? -360 : 360;
            diff = target - currentRotation;
        }
        return diff;
    }

    protected void autoDrive(double inches, double power, double timeoutSeconds) {
        autoDrive(inches, power, timeoutSeconds, new RunUntil() {
            @Override
            public boolean stop() {
                return false;
            }
        });
    }

    protected void autoDrive(double inches, double power, double timeoutSeconds, RunUntil runUntil) {
        for (Motor8696 motor : motors) {
            motor.storePosition();
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motor.setRelativeTarget((int) (-inches / (4 * Math.PI) * Motor8696.COUNTS_PER_REVOLUTION));

            motor.setPower(power);
        }

        runtime.reset();

        while (opModeIsActive() &&
                runtime.seconds() < timeoutSeconds &&
                Motor8696.motorsBusy(motors) &&
                !runUntil.stop()) {
            idle();
        }

        for (Motor8696 motor : motors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    protected void runMotors() {
        for (Motor8696 motor : motors) {
            motor.setPower();
        }
    }

    protected void enableEncoders() {
        for (Motor8696 motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Stores the gyro data into instance fields.
     */
    protected void getGyroData() {
        gravity = imu.getGravity();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }
}
