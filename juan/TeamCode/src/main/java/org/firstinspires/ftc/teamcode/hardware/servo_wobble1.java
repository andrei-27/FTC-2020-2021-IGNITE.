package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class servo_wobble1 {
    public Servo servo = null;

    public static double SERVO_RELEASEE = 0.86;

    public static double SERVO_CLOSEE = 0.44;

    public static double SERVO_MID = 0.73;

    public servo_wobble1(HardwareMap hwMap) {
        servo = hwMap.get(Servo.class, "servoWobbleBrat");
        up();
    }

    public void setServoPositions(double pos1) {
        if(pos1 > -1.0) {
            servo.setPosition(pos1);
        }
    }

    public void up() { setServoPositions(SERVO_RELEASEE); }

    public void mid() { setServoPositions(SERVO_MID); }

    public void down() { setServoPositions(SERVO_CLOSEE); }
}

