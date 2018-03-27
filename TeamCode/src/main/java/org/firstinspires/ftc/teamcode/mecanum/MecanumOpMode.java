package org.firstinspires.ftc.teamcode.mecanum;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.buttonEvents.Button;
import org.firstinspires.ftc.teamcode.buttonEvents.ButtonEvent;
import org.firstinspires.ftc.teamcode.Motor8696;
import org.firstinspires.ftc.teamcode.MotorState;
import org.firstinspires.ftc.teamcode.OpMode8696;
import org.firstinspires.ftc.teamcode.RunUntil;

/**
 * An implementation of the {@link OpMode8696} class to be used with the current 8696 robot.
 * This class includes features specific to the current robot that some robots may not have.
 */

public abstract class MecanumOpMode extends OpMode8696 {

    /**
     * Servo to push one of the jewels during autonomous <br/>
     * 0 = up, 1 = down
     * <br/>
     * "poosher" is a joke referencing one of the teachers at West High
     */
    protected Servo ballPoosher;

    /**
     * Servo to push a glyph inside the robot during teleOp <br/>
     * 0 = up, 1 = down
     */
    Servo cubePoosher;

    /**
     * Servo at the back left of the robot that rotates the platform for placing glyphs. <br/>
     * 1 = up, 0 = down
     */
    Servo leftDump;
    /**
     * Same as {@link #leftDump}, but on the right side. <br/>
     * 1 = up, 0 = down
     * @see #leftDump
     */
    Servo rightDump;

    protected DcMotor leftCollector;
    protected DcMotor rightCollector;

    DcMotor leftLift;
    DcMotor rightLift;

    /**
     * How many encoder counts the lift motors need
     * to rotate to elevate the lift by one unit.
     * Intended to be about a third of a glyph.
     */
    private final static int LIFT_BLOCK_HEIGHT = 1000;

    private int liftPosition = 0;

    private double liftOffsetLeft = 0;
    private double liftOffsetRight = 0;

    /**
     * constant for how many encoder counts are equivalent
     * to strafing one inch sideways.
     */
    private static final double STRAFE_COEFFICIENT = 94 * 10.5 / 18.5; // TODO: make this value more exact

    /**
     * Multiplier to alter the speed of the robot when the dpad is used to drive.
     */
    private static final double DPAD_MULT = 0.5;

    /**
     * The power used for the collectors.
     */
    private double collectorPower = 0.7;

    boolean collectorActive = false;

    int reverse = 1;
    private double reverseLeftCollector = 1;
    private double reverseRightCollector = 1;

    MotorState leftCollectorState;
    MotorState rightCollectorState;

    protected void initRobot() {
        super.initRobot();

        leftBack  .setDirection(DcMotor.Direction.FORWARD);
        rightBack .setDirection(DcMotor.Direction.REVERSE);
        leftFront .setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        for (Motor8696 motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        ballPoosher = hardwareMap.get(Servo.class, "ballPoosher");
        cubePoosher = hardwareMap.get(Servo.class, "cubePoosher");
        leftDump = hardwareMap.get(Servo.class, "leftDump");
        rightDump = hardwareMap.get(Servo.class, "rightDump");

        ballPoosher.setDirection(Servo.Direction.REVERSE);
        ballPoosher.scaleRange(0.3, 0.9);
        ballPoosher.setPosition(0); // set the servo to be against the robot right away.

        double min = 0.26;//0.24, 0.28

        double max = 0.76;
        scaleDump(min, max);

//        rightDump.setDirection(Servo.Direction.REVERSE);

        leftDump.setPosition(0);
        rightDump.setPosition(0);

        cubePoosher.scaleRange(0.17, 0.6);

        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");

        leftCollector = hardwareMap.get(DcMotor.class, "leftCollector");
        rightCollector = hardwareMap.get(DcMotor.class, "rightCollector");

        leftCollector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightCollector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightCollector.setDirection(DcMotorSimple.Direction.REVERSE);

        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftLift.setPower(0); // TODO: probably not useful code
        rightLift.setPower(0);

        leftCollectorState = new MotorState(leftCollector);
        rightCollectorState = new MotorState(rightCollector);
    }

    private void scaleDump(double min, double max) {
        leftDump.scaleRange(min, max);
        double add = -0.04;// less = down, more = up
        // -0.02 too high
        rightDump.scaleRange(min + add, max + add);
    }

    protected void initGyro() {
        telemetry.log().add("initializing imu...");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        boolean initialized = imu.initialize(parameters);

        telemetry.log().clear();
        telemetry.log().add(initialized ? "imu initialized successfully" : "imu failed to initialize");

    }

    void runCollector() {
        if (gamepad1.right_bumper) {
            leftCollector.setPower(-collectorPower);
            rightCollector.setPower(-collectorPower);
        } else if (collectorActive) {
            if (gamepad2.right_bumper) {
                leftCollector.setPower(-collectorPower);
                rightCollector.setPower(-collectorPower);
            } else if (gamepad2.b) {
                leftCollector.setPower(collectorPower);
                rightCollector.setPower(-collectorPower);
            } else if (gamepad2.x) {
                leftCollector.setPower(-collectorPower);
                rightCollector.setPower(collectorPower);
            } else {
                leftCollector.setPower(collectorPower * reverseLeftCollector);
                rightCollector.setPower(collectorPower * reverseRightCollector);
            }
        } else {
            leftCollector.setPower(0);
            rightCollector.setPower(0);
        }
    }

    /**
     * Check if either collector has jammed (slowed down sufficiently) and fix it.
     */
    public void checkCollectors() {
        reverseLeftCollector = reverseRightCollector = 1;
        if (checkCollector(leftCollectorState)) {
            reverseLeftCollector = -0.5;
        }
        else if (checkCollector(rightCollectorState)) {
            reverseRightCollector = -0.5;
        }
    }

    /**
     * Check if a specific collector has jammed (slowed down sufficiently) and fix it.
     */
    private boolean checkCollector(MotorState motorState) {
        telemetry.addData("", "%.2f", motorState.speed);
        if (motorState.power == 0 || motorState.lastPower <= 0)
            return false;
        if (motorState.speed < 0.1) {// TODO: find a more optimal value
            return true;
        }

        return false;
    }

    /**
     * Set the target position for both lift motors to begin running them.
     */
    private void setLiftTarget() {
        leftLift.setTargetPosition(liftPosition * LIFT_BLOCK_HEIGHT + (int) liftOffsetLeft);
        rightLift.setTargetPosition(liftPosition * LIFT_BLOCK_HEIGHT + (int) liftOffsetRight);

        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Run the lift motors in a way that keeps them synchronized and keeps motion smooth.
     */
    void runLift() {
        // calculate the encoder distance from the target position for both lift motors
        int leftDiff = Math.abs(liftPosition * LIFT_BLOCK_HEIGHT - leftLift.getCurrentPosition());
        int rightDiff = Math.abs(liftPosition * LIFT_BLOCK_HEIGHT - rightLift.getCurrentPosition());

        // plug that into a simple function and clip it to be < 1
        double leftPower = leftDiff * 0.05 + 0.25;
        double rightPower = rightDiff * 0.05 + 0.25;
        leftPower = Range.clip(leftPower, -1, 1);
        rightPower = Range.clip(rightPower, -1, 1);

        // calculate how much to slow one of the lift motors to keep them synchronized
        int diff = Math.abs(leftDiff - rightDiff);
        double slow = 1 - diff * 0.005;

        // make the one closer to the target be slower
        if (leftDiff > rightDiff) {
            rightPower *= slow;
        } else if (leftDiff > rightDiff) {
            leftPower *= slow;
        }

        leftLift.setPower(leftPower);
        rightLift.setPower(rightPower);
    }

    protected void alignLift() {
        boolean sleep = false;
        if (gamepad2.left_stick_button) {
            liftOffsetLeft -= gamepad2.left_stick_y * 20;
            sleep = true;
        }

        if (gamepad2.right_stick_button) {
            liftOffsetRight -= gamepad2.right_stick_y * 20;
            sleep = true;
        }

        if (sleep) {
            setLiftTarget();
            sleep(1);
        }
    }

    /**
     * Assign button events at the start of the program
     */
    void buttonEvents() {
        addButtonEvent(1, new ButtonEvent(Button.X) {
            public void onDown() {
                reverse *= -1;
            }
        });

        addButtonEvent(1, new ButtonEvent(Button.LEFT_BUMPER) {
            public void onDown() {
                controlDriveSpeed();
            }
            public void onUp() {
                controlDriveSpeed();
            }
        });

        addButtonEvent(2, new ButtonEvent(Button.A) {
            public void onDown() {
                collectorActive = !collectorActive;
            }
        });

        addButtonEvent(2, new ButtonEvent(Button.LEFT_BUMPER) {
            public void onDown() {
                cubePoosher.setPosition(0);
            }

            public void onUp() {
                cubePoosher.setPosition(1);
            }
        });

        addButtonEvent(2, new ButtonEvent(Button.Y) {
            public void onDown() {
                leftDump.setPosition(1);
                rightDump.setPosition(1);
            }

            public void onUp() {
                leftDump.setPosition(0);
                rightDump.setPosition(0);
            }
        });

        addButtonEvent(2, new ButtonEvent(Button.UP) {
            public void onDown() {
                liftPosition++;
                setLiftTarget();
//                scaleDump(0.3, 0.64);
//                idle();
//                leftDump.setPosition(leftDump.getPosition());
//                rightDump.setPosition(rightDump.getPosition());
            }
        });

        addButtonEvent(2, new ButtonEvent(Button.DOWN) {
            public void onDown() {
                liftPosition--;
                setLiftTarget();
//                scaleDump(0.24, 0.64);
//                idle();
//                leftDump.setPosition(leftDump.getPosition());
//                rightDump.setPosition(rightDump.getPosition());
            }
        });
    }

    private void controlDriveSpeed() {
        driveSpeed = 1;

//        if (gamepad1.right_bumper)
//            driveSpeed *= 0.5;

        if (gamepad1.left_bumper)
            driveSpeed *= 0.4;
    }

    boolean dpadDrive() {
        if (gamepad1.dpad_right) {
            horizontalDrive(DPAD_MULT * driveSpeed);
        } else if (gamepad1.dpad_left) {
            horizontalDrive(-DPAD_MULT * driveSpeed);
        } else if (gamepad1.dpad_up) {
            verticalDrive(DPAD_MULT * driveSpeed / Math.sqrt(2), -1, -1);
        } else if (gamepad1.dpad_down) {
            verticalDrive(DPAD_MULT * driveSpeed / Math.sqrt(2), 1, 1);
        } else
            return false;
        runMotors();
        return true;
    }

    /**
     * Run the vertical component of driving with mecanum wheels.
     * Tank driving combined with lateral motion.
     * @param mult power multiplier
     * @param left left side power
     * @param right right side power
     *
     * @see #horizontalDrive(double)
     */
    void verticalDrive(double mult, double left, double right) {
        leftFront .addPower(mult * left);
        leftBack  .addPower(mult * left);
        rightFront.addPower(mult * right);
        rightBack .addPower(mult * right);
    }

    /**
     * Run the horizontal component of driving with mecanum wheels.
     * Tank driving combined with lateral motion.
     * @param power the power delivered to the motors.
     *
     * @see #verticalDrive(double, double, double)
     */
    void horizontalDrive(double power) {
        leftFront .addPower(-power);
        leftBack  .addPower( power);
        rightFront.addPower( power);
        rightBack .addPower(-power);
    }

    /**
     * Calculate the angles for driving and move/rotate the robot.
     * Currently unused because the other driving method is better.
     */
    protected void mecanumTeleOpDrive() {
        double ly = gamepad1.left_stick_y;
        double lx = gamepad1.left_stick_x;
        double ry = gamepad1.right_stick_y;
        double rx = gamepad1.right_stick_x;

        double leftAngle  = getGamepadAngle(lx, ly);

        double rightAngle = getGamepadAngle(rx, ry);

        for (Motor8696 motor : motors) {
            motor.reset();
        }

        getGyroData();

        if (!Double.isNaN(leftAngle) && getMagnitude(lx, ly) >= 0.25) {
            driveDirection(leftAngle * 180 / Math.PI, getMagnitude(lx, ly));
        }

        if (!Double.isNaN(rightAngle) && getMagnitude(rx, ry) >= 0.5) {
            telemetry.addData("angle", rightAngle * 180 / Math.PI - 90);
            onHeading(rightAngle * 180 / Math.PI - 90, 0.5, 1, false);
            for (Motor8696 motor : motors)
                motor.addPower(0, 1);
        }

        runMotors();
    }

    /**
     * Drive the robot in a specified direction regardless of its
     * current rotation. Meant to be used with mecanum wheels.
     *
     * @param angle degrees, angle that the robot should move towards. Starts at standard position.
     * @param power number to scale all the motor power by.
     */
    private void driveDirection(double angle, double power) {
        angle = adjustAngle(angle, angles.firstAngle);
        driveDirectionRelative(angle, power);
    }

    /**
     * Drive the robot in a specific direction relative
     * to the robot orientation.
     *
     * @param angle degrees, angle that the robot should move towards. Starts at standard position.
     * @param power number to scale all the motor power by.
     */
    private void driveDirectionRelative(double angle, double power) {
        angle = angle / 180 * Math.PI;
        leftBack  .addPower((Math.sin(angle) + Math.cos(angle)) * power);
        rightBack .addPower((Math.sin(angle) - Math.cos(angle)) * power);
        leftFront .addPower((Math.sin(angle) - Math.cos(angle)) * power);
        rightFront.addPower((Math.sin(angle) + Math.cos(angle)) * power);
    }

    protected void autoStrafe(double inches, double power, double timeoutSeconds) {
        autoStrafe(inches, power, timeoutSeconds, new RunUntil() {
            @Override
            public boolean stop() {
                return false;
            }
        });
    }

    /**
     * Strafe to the side a set distance
     *
     * @param inches Number of inches to strafe.
     *               positive is right, negative is left.
     * @param power Power to set the motors to.
     * @param timeoutSeconds After this many seconds, it stops trying.
     */
    protected void autoStrafe(double inches, double power, double timeoutSeconds, RunUntil runUntil) {
        for (Motor8696 motor : motors) {
            motor.storePosition();
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        leftBack  .setRelativeTarget((int) ( inches * STRAFE_COEFFICIENT));
        rightBack .setRelativeTarget((int) (-inches * STRAFE_COEFFICIENT));
        leftFront .setRelativeTarget((int) (-inches * STRAFE_COEFFICIENT));
        rightFront.setRelativeTarget((int) ( inches * STRAFE_COEFFICIENT));

        for (Motor8696 motor : motors)
            motor.setPower(Math.abs(power));

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

    /**
     * {@inheritDoc}
     */
    @Override
    protected void periodic() {
//        robotState.update(this);
        leftCollectorState.update();
        rightCollectorState.update();
    }
}
