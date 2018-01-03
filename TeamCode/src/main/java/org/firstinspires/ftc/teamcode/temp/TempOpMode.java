package org.firstinspires.ftc.teamcode.temp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Button;
import org.firstinspires.ftc.teamcode.ButtonEvent;
import org.firstinspires.ftc.teamcode.Motor8696;
import org.firstinspires.ftc.teamcode.OpMode8696;

/**
 * Created by USER on 11/5/2017.
 */

public abstract class TempOpMode extends OpMode8696 {
    DcMotor cubeLinearArm;
    Servo grabber1;
    Servo grabber2;
    Servo ballPoosher;

    boolean servosActive = true;

    private static final double DPAD_MULT = 0.5;

    int reverse = 1;


//    ColorSensor tapeColorSensor;

    ColorSensor ballColorSensor;

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

        cubeLinearArm = hardwareMap.get(DcMotor.class, "cubeArm");
        grabber1 = hardwareMap.get(Servo.class, "grabber1");
        grabber2 = hardwareMap.get(Servo.class, "grabber2");
        ballPoosher = hardwareMap.get(Servo.class, "ballPoosher");
        ballColorSensor = hardwareMap.get(ColorSensor.class, "color"); // TODO: change config name
//        tapeColorSensor = hardwareMap.get(ColorSensor.class, "tapeColor");

//        ballColorSensor.setI2cAddress(I2cAddr.create7bit(0x3c));

//        ballColorSensor.resetDeviceConfigurationForOpMode();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("color", ballColorSensor.getConnectionInfo());
        telemetry.update();

        grabber2.setDirection(Servo.Direction.REVERSE);

        grabber1.scaleRange(0.5, 0.8);
        grabber2.scaleRange(0.2, 0.5);

        ballPoosher.scaleRange(0.0, 0.8); //TODO: set correct values

        grabber1.setPosition(1);
        grabber2.setPosition(1);
        ballPoosher.setPosition(0);

        ballColorSensor.enableLed(false);
        ballColorSensor.enableLed(true);

        cubeLinearArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    void buttonEvents() {
        addButtonEvent(1, new ButtonEvent(Button.LEFT_BUMPER) {
            public void onDown() {
                driveSpeed = 0.5;
            }

            public void onUp() {
                driveSpeed = 1;
            }
        });

        addButtonEvent(2, new ButtonEvent(Button.A) {
            boolean grabbing = true;

            public void onDown() {
                grabbing = !grabbing;

                grabber1.setPosition(grabbing ? 1 : 0);
                grabber2.setPosition(grabbing ? 1 : 0);

                if (!servosActive) {
                    grabber1.getController().pwmEnable();
                    servosActive = true;
                }
            }
        });

        addButtonEvent(2, new ButtonEvent(Button.X) {
            double lastPos = 1;

            public void onDown() {
                if (grabber1.getPosition() == 0.5) {
                    grabber1.setPosition(lastPos);
                    grabber2.setPosition(lastPos);
                } else {
                    lastPos = grabber1.getPosition();
                    grabber1.setPosition(0.5);
                    grabber2.setPosition(0.5);
                }

                if (!servosActive) {
                    grabber1.getController().pwmEnable();
                    servosActive = true;
                }
            }
        });

        addButtonEvent(2, new ButtonEvent(Button.LEFT_BUMPER) {
            private boolean down = true;

            public void onDown() {
                ballPoosher.setPosition(down ? 0.7 : 0);
                down = !down;

                if (!servosActive) {
                    grabber1.getController().pwmEnable();
                    servosActive = true;
                }
            }
        });

        addButtonEvent(2, new ButtonEvent(Button.B) {

            public void onDown() {
                if (servosActive) {
                    grabber1.getController().pwmDisable();
                } else {
                    grabber1.getController().pwmEnable();
                }

                servosActive = !servosActive;
            }
        });

        addButtonEvent(1, new ButtonEvent(Button.X) {
            public void onDown() {
                reverse *= -1;
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

    void pushBall(boolean color) {
        boolean foundBall = false;
        boolean which = false;

        for (double pos = 0.2; pos < 1; pos += 0.01) {
            ballPoosher.setPosition(pos);
            sleep(10);

            if (ballColorSensor.red() > 3 || ballColorSensor.blue() > 3) {
                foundBall = true;
                which = ballColorSensor.red() > ballColorSensor.blue();
                break;
            }
        }

        if (foundBall) {
            if (which == color) {
                ghettoTurn(-0.25);
            } else {
                ghettoTurn(0.25);
            }

            ballPoosher.setPosition(1);

//            autoTurn(0, 0.35, 5);
        } else {
            telemetry.log().add("ball not found");
        }
    }

    void ghettoTurn(double power) {
        leftFront.setPower(-power);
        leftBack.setPower(-power);
        rightFront.setPower(power);
        rightBack.setPower(power);

        sleep(250);

        for (Motor8696 motor : motors)
            motor.setPower(0);
    }
}
