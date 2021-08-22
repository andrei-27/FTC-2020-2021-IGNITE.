package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class servo_intake {
    public Servo servo = null;

    public static double SERVO_DOWN = 0.16;

    public static double SERVO_UP = 0.58;

    public servo_intake(HardwareMap hwMap) {
        servo = hwMap.get(Servo.class, "servoIntake");
        up();
    }

    public void setServoPositions(double pos1) {
        if(pos1 > -1.0) {
            servo.setPosition(pos1);
        }
    }

    public void down() { setServoPositions(SERVO_DOWN); }

    public void up() {
        setServoPositions(SERVO_UP);
    }
}

