package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by USER on 12/13/2017.
 */

public class ServoTarget {
    private Servo servo;
    private long startTime;
    private long endTime;
    private double startPos;
    private double endPos;

    public static final ServoTarget NULL = new ServoTarget(null, 0, 0, 0, 0) {
        @Override
        public boolean runServo() {
            return true;
        }
    };

    public ServoTarget(Servo servo, long startTime, long endTime, double startPos, double endPos) {
        this.servo = servo;
        this.startTime = startTime;
        this.endTime = endTime;
        this.startPos = startPos;
        this.endPos = endPos;
    }

    /**
     * move the servo to where it should be at a time.
     *
     * @return Whether or not the servo is still running.
     */
    public boolean runServo() {
        if (System.currentTimeMillis() >= endTime) {
            servo.setPosition(endPos);
            return false;
        }

        int currentTime = (int) (System.currentTimeMillis() - startTime);
        int timeDiff = (int) (endTime - startTime);

        double posDiff = endPos - startPos;

        double setPos = (double) currentTime / timeDiff * posDiff + startPos;

        servo.setPosition(setPos);

        return true;
    }
}
