package org.firstinspires.ftc.teamcode.mecanum;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Button;
import org.firstinspires.ftc.teamcode.ButtonEvent;
import org.firstinspires.ftc.teamcode.Motor8696;
import org.firstinspires.ftc.teamcode.OpMode8696;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.RunUntil;
import org.firstinspires.ftc.teamcode.ServoTarget;

/**
 * Created by USER on 11/5/2017.
 */

public abstract class MecanumOpMode extends OpMode8696 {

    Servo ballPoosher;
    Servo cubePoosher;
    Servo leftDump;
    Servo rightDump;

    DcMotor leftCollector;
    DcMotor rightCollector;

    DcMotor leftLift;
    DcMotor rightLift;

    /**
     * constant for how many encoder counts are equivalent
     * to strafing one inch sideways.
     */
    private static final double STRAFE_COEFFICIENT = 400; // TODO: make this value more exact

    private static final double DPAD_MULT = 0.5;

    boolean collectorActive = false;

    int reverse = 1;

    ServoTarget cubePoosherTarget = ServoTarget.NULL;

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

        double min = 0.19;
        double max = 0.72;
        leftDump.scaleRange(min, max);
        double add = 0; // less = down, more = up
//        rightDump.scaleRange(1 - max + add + 0.05, 1 - min + add);
        rightDump.scaleRange(min + add, max + add);

//        rightDump.setDirection(Servo.Direction.REVERSE);

        leftDump.setPosition(0);
        rightDump.setPosition(0);

        cubePoosher.scaleRange(0.4, 0.9);

        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");

        leftCollector = hardwareMap.get(DcMotor.class, "leftCollector");
        rightCollector = hardwareMap.get(DcMotor.class, "rightCollector");

        leftCollector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightCollector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftCollector.setDirection(DcMotorSimple.Direction.REVERSE);

        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robotState = new RobotState(this, new DcMotor[]{
                motors[0].getMotor(),
                motors[1].getMotor(),
                motors[2].getMotor(),
                motors[3].getMotor(),
                leftCollector,
                rightCollector
        });
    }

    void buttonEvents() {
        addButtonEvent(1, new ButtonEvent(Button.X) {
            public void onDown() {
                reverse *= -1;
            }
        });

        addButtonEvent(1, new ButtonEvent(Button.LEFT_BUMPER) {
            public void onDown() {
                driveSpeed = 0.3;
            }

            public void onUp() {
                driveSpeed = 1;
            }
        });

        addButtonEvent(2, new ButtonEvent(Button.A) {
            public void onDown() {
                collectorActive = !collectorActive;
            }
        });

        addButtonEvent(2, new ButtonEvent(Button.LEFT_BUMPER) {
            private static final double SPEED = 0.001;
            private double min = 0.4, max = 0.9; // TODO: put this in one place

            public void onDown() {
                double target = 0;
                double diff = Math.abs(cubePoosher.getPosition() - target) * (max - min);

                cubePoosherTarget = new ServoTarget(cubePoosher,
                        System.currentTimeMillis(),
                        System.currentTimeMillis() + (int) (diff / SPEED),
                        cubePoosher.getPosition(),
                        target);
            }

            public void onUp() {
                cubePoosherTarget = ServoTarget.NULL;
                cubePoosher.setPosition(1);
            }
        });

        addButtonEvent(1, new ButtonEvent(Button.Y) {
            public void onDown() {
                cubePoosher.setPosition(0.5);
            }

            public void onUp() {
                cubePoosher.setPosition(1);
            }
        });

        addButtonEvent(2, new ButtonEvent(Button.Y) {
            public void onDown() {
                leftDump.setPosition(1);
                rightDump.setPosition(1);
                cubePoosher.setPosition(0.5);
            }

            public void onUp() {
                leftDump.setPosition(0);
                rightDump.setPosition(0);
                cubePoosher.setPosition(1);
            }
        });
    }

    boolean dpadDrive() {
        if (gamepad1.dpad_right) {
            horizontalDrive(DPAD_MULT * driveSpeed, 1);
        } else if (gamepad1.dpad_left) {
            horizontalDrive(DPAD_MULT * driveSpeed, -1);
        } else if (gamepad1.dpad_up) {
            verticalDrive(DPAD_MULT * driveSpeed / Math.sqrt(2), -1, -1);
        } else if (gamepad1.dpad_down) {
            verticalDrive(DPAD_MULT * driveSpeed / Math.sqrt(2), 1, 1);
        } else
            return false;
        runMotors();
        return true;
    }

    void verticalDrive(double mult, double left, double right) {
        leftFront .addPower(mult * left);
        leftBack  .addPower(mult * left);
        rightFront.addPower(mult * right);
        rightBack .addPower(mult * right);
    }

    void horizontalDrive(double mult, double value) {
        leftFront .addPower(mult * -(value));
        leftBack  .addPower(mult *  (value));
        rightFront.addPower(mult *  (value));
        rightBack .addPower(mult * -(value));
    }

    /**
     * Calculate the angles for driving and move/rotate the robot.
     */
    protected void mecanumTeleOpDrive() {
        double ly = gamepad1.left_stick_y;
        double lx = -gamepad1.left_stick_x;
        double ry = gamepad1.right_stick_y;
        double rx = -gamepad1.right_stick_x;

        double leftAngle  = getGamepadAngle(lx, ly);

        double rightAngle = getGamepadAngle(rx, ry);

        for (Motor8696 motor : motors) {
            motor.reset();
        }

        getGyroData();

        if (!Double.isNaN(leftAngle) && getMagnitude(lx, ly) >= 0.25) {
            driveDirectionRelativeToRobot(leftAngle, getMagnitude(lx, ly));
        }

        if (!Double.isNaN(rightAngle) && getMagnitude(rx, ry) >= 0.5) {
            onHeading(rightAngle * 180 / Math.PI, 0.5, 1, false);
            for (Motor8696 motor : motors)
                motor.addPower(0, 1);
        }

        runMotors();
    }

    /**
     * Drive the robot in a specified direction regardless of its
     * current rotation. Meant to be used with mecanum wheels.
     *
     * @param angle angle that the robot should move towards. Starts at standard position.
     * @param power number to scale all the motor power by.
     */
    private void driveDirectionRelativeToRobot(double angle, double power) {
        angle = adjustAngle(angle, angles.firstAngle);
        angle = angle / 180 * Math.PI;
        leftBack  .addPower((Math.sin(angle) - Math.cos(angle)) * power);
        rightBack .addPower((Math.sin(angle) + Math.cos(angle)) * power);
        leftFront .addPower((Math.sin(angle) + Math.cos(angle)) * power);
        rightFront.addPower((Math.sin(angle) - Math.cos(angle)) * power);
    }

    void autoStrafe(double inches, double power, double timeoutSeconds) {
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
    void autoStrafe(double inches, double power, double timeoutSeconds, RunUntil runUntil) {
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

    @Override
    protected void periodic() {
        robotState.update(this);
    }
}
