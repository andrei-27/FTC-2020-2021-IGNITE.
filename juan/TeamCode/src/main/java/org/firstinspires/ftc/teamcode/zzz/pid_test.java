package org.firstinspires.ftc.teamcode.zzz;

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
import org.firstinspires.ftc.teamcode.zzz.init_robot;
import org.firstinspires.ftc.teamcode.hardware.servo_glisiera;
import org.firstinspires.ftc.teamcode.hardware.servo_outtake1;
import org.firstinspires.ftc.teamcode.hardware.servo_outtake2;
import org.firstinspires.ftc.teamcode.hardware.servo_plug;
import org.firstinspires.ftc.teamcode.hardware.servo_wobble1;
import org.firstinspires.ftc.teamcode.hardware.servo_wobble2;


@Config
@TeleOp
@Disabled
public class pid_test extends LinearOpMode {

    init_robot conserva = new init_robot();

    private double root2 = Math.sqrt(2.0);

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    Boolean cont_glisiera = Boolean.FALSE;

    DcMotorEx outtake;
    DcMotorEx intake;

    public static double NEW_P = 61;
    public static double NEW_I = 12;
    public static double NEW_D = 11;
    public static double NEW_F = 15.6;
    public static double valoare = 1480;


    public void runOpMode() {
        // get reference to DC motor.
        // since we are using the Expansion Hub,
        // cast this motor to a DcMotorEx object.

        conserva.init(hardwareMap);

        outtake = (DcMotorEx)hardwareMap.get(DcMotor.class, "outtake");
        intake = (DcMotorEx)hardwareMap.get(DcMotor.class, "intake");

        Gamepad gp1 = gamepad1;

        servo_outtake1 out1 = new servo_outtake1(hardwareMap);
        servo_outtake2 out2 = new servo_outtake2(hardwareMap);
        servo_wobble1 wob_brat = new servo_wobble1(hardwareMap);
        servo_wobble2 wob_cleste = new servo_wobble2(hardwareMap);
        servo_glisiera outg = new servo_glisiera(hardwareMap);
        servo_plug plug = new servo_plug(hardwareMap);
        out1.open();
        out2.open();
        outg.open();
        wob_brat.mid();
        wob_cleste.close();
        plug.up();

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

            double direction = Math.atan2(-gp1.left_stick_y, gp1.left_stick_x) - Math.PI/2;
            double rotation = -gp1.right_stick_x;
            double speed = Math.sqrt(gp1.left_stick_x*gp1.left_stick_x + gp1.left_stick_y*gp1.left_stick_y);

            setDrivePowers(direction, Math.pow(speed, 3.0),0.75*Math.pow(rotation, 3.0));


            if (gp1.left_bumper && !cont_glisiera){
                outtake.setVelocity(valoare);
                out1.close();
                out2.close();
                sleep(250);
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

            if(gp1.a){
                plug.down();
            }
            else{
                plug.up();
            }

            if (gp1.left_trigger != 0){
                outtake.setVelocity(valoare);
            }
            if (gp1.left_trigger == 0){
                outtake.setVelocity(0);
            }

            intake.setPower(gp1.right_trigger);

            dashboardTelemetry.addData("velocity", outtake.getVelocity());
            dashboardTelemetry.addData("target velocity", valoare);
            dashboardTelemetry.update();
        }
    }

    public void setDrivePowers(double direction, double speed, double rotateSpeed){
        double directionRads = direction;

        double sin = Math.sin(Math.PI/4 - directionRads);
        double cos = Math.cos(Math.PI/4 - directionRads);

        conserva.lf.setPower(-root2 * speed * sin + rotateSpeed);
        conserva.rf.setPower(-root2 * speed * cos - rotateSpeed);
        conserva.lr.setPower(-root2 * speed * cos + rotateSpeed);
        conserva.rr.setPower(-root2 * speed * sin - rotateSpeed);
    }
}