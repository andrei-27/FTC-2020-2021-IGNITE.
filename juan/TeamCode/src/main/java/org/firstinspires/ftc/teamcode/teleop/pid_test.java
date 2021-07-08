package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.servo_glisiera;
import org.firstinspires.ftc.teamcode.hardware.servo_outtake1;
import org.firstinspires.ftc.teamcode.hardware.servo_outtake2;
import org.firstinspires.ftc.teamcode.hardware.servo_wobble1;
import org.firstinspires.ftc.teamcode.hardware.servo_wobble2;


@Config
@TeleOp
@Disabled
public class pid_test extends LinearOpMode {


    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    Boolean cont_glisiera = Boolean.FALSE;

    DcMotorEx outtake;
    DcMotorEx intake;

    public static double NEW_P = 61;
    public static double NEW_I = 0.7;
    public static double NEW_D = 11;
    public static double NEW_F = 15.6;
    public static double valoare = 1480;

    double lowestSpeed = 2000.0;
    double highestSpeed = 100.0;

    public void runOpMode() {
        // get reference to DC motor.
        // since we are using the Expansion Hub,
        // cast this motor to a DcMotorEx object.
        outtake = (DcMotorEx)hardwareMap.get(DcMotor.class, "outtake");
        intake = (DcMotorEx)hardwareMap.get(DcMotor.class, "intake");

        Gamepad gp1 = gamepad1;
        Gamepad gp2 = gamepad2;

        servo_outtake1 out1 = new servo_outtake1(hardwareMap);
        servo_outtake2 out2 = new servo_outtake2(hardwareMap);
        servo_wobble1 wob_brat = new servo_wobble1(hardwareMap);
        servo_wobble2 wob_cleste = new servo_wobble2(hardwareMap);
        servo_glisiera outg = new servo_glisiera(hardwareMap);
        out1.open();
        out2.open();
        outg.open();
        wob_brat.mid();
        wob_cleste.close();

        // wait for start command.
        waitForStart();


        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotor.Direction.FORWARD);

        intake.setPower(0.0);



        // get the PID coefficients for the RUN_USING_ENCODER  modes.
        PIDFCoefficients pidOrig = outtake.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // change coefficients using methods included with DcMotorEx class.
        PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        outtake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

        // re-read coefficients and verify change.
        PIDCoefficients pidModified = outtake.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // display info to user.
        while(opModeIsActive()) {


            if (gp1.left_bumper && !cont_glisiera){
                outtake.setVelocity(valoare);
                out1.close();
                out2.close();
                sleep(250);
                lowestSpeed = 2000.0;
                highestSpeed = 100.0;
                outg.close();
                cont_glisiera = true;
            }
            if (!gp1.left_bumper && cont_glisiera){
                outg.open();
                sleep(450);
                out1.open();
                out2.open();
                cont_glisiera = false;
                outtake.setVelocity(0);
            }


            if (gp1.left_trigger != 0){
                outtake.setVelocity(valoare);
            }
            if (gp1.left_trigger == 0){
                outtake.setVelocity(0);
            }

            intake.setPower(gp1.right_trigger);


            lowestSpeed = Math.min(lowestSpeed, outtake.getVelocity());
            highestSpeed = Math.max(highestSpeed, outtake.getVelocity());

            telemetry.addData("velocity", outtake.getVelocity());
            telemetry.addData("target velocity", valoare);
            telemetry.addData("lowest velocity", lowestSpeed);
            telemetry.addData("highest velocity", highestSpeed);
            telemetry.update();
        }
    }
}