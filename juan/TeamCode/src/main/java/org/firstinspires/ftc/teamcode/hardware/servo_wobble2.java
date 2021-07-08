package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class servo_wobble2 {
    public Servo servo = null;

    public static double SERVO_RELEASEE = 0.7;

    public static double SERVO_CLOSEE = 0.11;

    public servo_wobble2(HardwareMap hwMap) {
        servo = hwMap.get(Servo.class, "servoWobbleCleste");
        close();
    }

    public void setServoPositions(double pos1) {
        if(pos1 > -1.0) {
            servo.setPosition(pos1);
        }
    }

    public void open() { setServoPositions(SERVO_RELEASEE); }

    public void close() {
        setServoPositions(SERVO_CLOSEE);
    }
}

