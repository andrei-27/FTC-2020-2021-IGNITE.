package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class servo_plug {
    public Servo servo = null;

    public static double SERVO_DOWN = 0.62;

    public static double SERVO_UP = 0.13;

    public servo_plug(HardwareMap hwMap) {
        servo = hwMap.get(Servo.class, "servoPlug");
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

