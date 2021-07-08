package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class servo_outtake2 {
    public Servo servoo = null;

    public static double SERVO_RELEASE = 0.55;

    public static double SERVO_CLOSE = 0.80;

    public servo_outtake2(HardwareMap hwMap) {
        servoo = hwMap.get(Servo.class, "servoOuttake2");
        close();
    }

    public void setServoPositions(double pos1) {
        if(pos1 > -1.0) {
            servoo.setPosition(pos1);
        }
    }

    public void open() { setServoPositions(SERVO_RELEASE); }

    public void close() {
        setServoPositions(SERVO_CLOSE);
    }
}

