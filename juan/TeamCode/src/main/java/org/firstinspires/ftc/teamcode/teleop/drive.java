/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.hardware.init_robot;
import org.firstinspires.ftc.teamcode.hardware.servo_outtake1;
import org.firstinspires.ftc.teamcode.hardware.servo_outtake2;
import org.firstinspires.ftc.teamcode.hardware.servo_glisiera;
import org.firstinspires.ftc.teamcode.hardware.servo_wobble1;
import org.firstinspires.ftc.teamcode.hardware.servo_wobble2;
import org.firstinspires.ftc.teamcode.hardware.servo_plug;

import javax.xml.parsers.FactoryConfigurationError;

import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;


@TeleOp
@Disabled
public class drive extends LinearOpMode {

    init_robot conserva = new init_robot();

    private double root2 = Math.sqrt(2.0);
    Boolean ok3 = FALSE;
    Boolean ok2 = FALSE;
    Boolean cont = FALSE;
    Boolean ok = FALSE;
    Boolean contor = FALSE;
    Boolean cont_glisiera = FALSE;
    Boolean slow_mode = FALSE;

    //pula mea

    public static double NEW_P = 38;
    public static double NEW_I = 14;
    public static double NEW_D = 20;
    public double HIGH_VELO = 1400;

    @Override
    public void runOpMode() {

        DcMotorEx outtake = null; // Intake motor
        outtake = (DcMotorEx)hardwareMap.get(DcMotor.class, "outtake");
        PIDCoefficients pidOrig = outtake.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDCoefficients pidNew = new PIDCoefficients(NEW_P, NEW_I, NEW_D);
        outtake.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        PIDCoefficients pidModified = outtake.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);


        Gamepad gp1 = gamepad1;
        Gamepad gp2 = gamepad2;
		
        conserva.init(hardwareMap);

        servo_outtake1 out1 = new servo_outtake1(hardwareMap);
        servo_outtake2 out2 = new servo_outtake2(hardwareMap);
        servo_wobble1 wob_brat = new servo_wobble1(hardwareMap);
        servo_wobble2 wob_cleste = new servo_wobble2(hardwareMap);
        servo_plug plug = new servo_plug(hardwareMap);
        servo_glisiera outg = new servo_glisiera(hardwareMap);
        out1.open();
        out2.open();
        outg.open();
        wob_brat.mid();
        wob_cleste.close();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while(opModeIsActive()) {


            /* gamepad 1 */

            /* Drive */
			
			double direction = Math.atan2(-gp1.left_stick_y, gp1.left_stick_x) - Math.PI/2;
			double rotation = -gp1.right_stick_x;
			double speed = Math.sqrt(gp1.left_stick_x*gp1.left_stick_x + gp1.left_stick_y*gp1.left_stick_y);

            if(gp1.left_bumper){
                slow_mode = true;
            }
            if(gp1.right_bumper){
                slow_mode = false;
            }
            if(slow_mode){
                setDrivePowers(direction, 0.38 *Math.pow(speed, 3.0), 0.35*Math.pow(rotation, 3.0));
            }
            else{
                setDrivePowers(direction, Math.pow(speed, 3.0),0.75*Math.pow(rotation, 3.0));
            }



            /* gamepad 2 */

            if (gp2.left_bumper && !cont_glisiera){
                outtake.setVelocity(HIGH_VELO);
                out1.close();
                out2.close();
                sleep(250);
                outg.close();
                cont_glisiera = true;
            }
            if (!gp2.left_bumper && cont_glisiera){
                outg.open();
                sleep(400);
                out1.open();
                out2.open();
                cont_glisiera = false;
                outtake.setVelocity(0);
            }


            if(gp2.x){
                wob_brat.down();
            }
            else{
                wob_brat.mid();
            }
            if(gp2.y){
                wob_cleste.open();
            }
            else{
                wob_cleste.close();
            }
            if(gp2.a){
                plug.down();
            }
            else{
                plug.up();
            }


            

            if (gp2.dpad_up && !cont){
                HIGH_VELO = HIGH_VELO + 20;
                cont = !cont;
            }
            if (gp2.dpad_down && !contor){
                HIGH_VELO = HIGH_VELO - 20;
                contor = !contor;
            }
            if (!gp2.dpad_up){
                cont = false;
            }
            if (!gp2.dpad_down){
                contor = false;
            }


            if(gp2.right_trigger> 0) {
                ok2 = !ok2;
            }
            else{
                ok2 = false;
            }

            if(ok2){
                outtake.setVelocity(-480);
            }else{
                outtake.setVelocity(Math.min(gp2.left_trigger*2000, HIGH_VELO));
            }

            if(gp2.right_bumper) {
                ok3 = true;
            }
            else{
                ok3 = false;
            }

            if(ok3){
                conserva.intake.setPower(-0.8);
            }else{
                conserva.intake.setPower(Math.min(gp2.right_trigger, 0.9));
            }


            telemetry.addData("outtake velocity", HIGH_VELO);
            telemetry.addData("slow_mode", slow_mode);
            telemetry.update();
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
