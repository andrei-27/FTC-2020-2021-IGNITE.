package org.firstinspires.ftc.teamcode.teleop.augmentedDrive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.servo_glisiera;
import org.firstinspires.ftc.teamcode.hardware.servo_outtake1;
import org.firstinspires.ftc.teamcode.hardware.servo_outtake2;
import org.firstinspires.ftc.teamcode.hardware.servo_plug;
import org.firstinspires.ftc.teamcode.hardware.servo_wobble1;
import org.firstinspires.ftc.teamcode.hardware.servo_wobble2;

import static java.lang.Boolean.FALSE;


@TeleOp
public class driveRemoteRosu extends LinearOpMode {


    private double root2 = Math.sqrt(2.0);
    Boolean ok3 = FALSE;
    Boolean ok2 = FALSE;
    Boolean cont = FALSE;
    Boolean contor = FALSE;
    Boolean cont_glisiera = FALSE;


    public static double NEW_P = 61;
    public static double NEW_I = 0.7;
    public static double NEW_D = 11;
    public static double NEW_F = 15.6;
    public double HIGH_VELO = 1480;
    public double POWERSHOT_VELO = 1300;

    // Define 2 states, drive control or automatic control
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;

    // The coordinates we want the bot to automatically go to when we press the A button
    Vector2d targetAVector;
    // The heading we want the bot to end on for targetA
    double targetAHeading;

    // The angle we want to align to when we press Y
    double targetAngle = Math.toRadians(0);

    SampleMecanumDrive drive;

    Vector2d towerVector = new Vector2d(125, 31);

    @Override
    public void runOpMode() {
        // Initialize custom cancelable SampleMecanumDrive class

        DcMotorEx outtake = null; // Intake motor
        outtake = (DcMotorEx)hardwareMap.get(DcMotor.class, "outtake");

        PIDFCoefficients pidOrig = outtake.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // change coefficients using methods included with DcMotorEx class.
        PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        outtake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

        // re-read coefficients and verify change.
        PIDCoefficients pidModified = outtake.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);


        DcMotor intake = null;
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setPower(0.0);



        servo_outtake1 out1 = new servo_outtake1(hardwareMap);
        servo_outtake2 out2 = new servo_outtake2(hardwareMap);
        servo_wobble1 wob_brat = new servo_wobble1(hardwareMap);
        servo_wobble2 wob_cleste = new servo_wobble2(hardwareMap);
        servo_glisiera outg = new servo_glisiera(hardwareMap);
        servo_plug plug = new servo_plug(hardwareMap);
        plug.up();
        out1.open();
        out2.open();
        outg.open();
        wob_brat.mid();
        wob_cleste.close();



        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Update the drive class
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Print pose to telemetry


            // We follow different logic based on whether we are in manual driver control or switch
            // control to the automatic mode
            switch (currentMode) {
                case DRIVER_CONTROL:
                    drive.setDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            )
                    );

                    if(gamepad1.a)
                        resetPositionCorner();

                    if(gamepad1.b)
                        resetPositionLine();

                    if (gamepad1.right_bumper) {
                        targetAngle = Math.atan2(-poseEstimate.getY() + towerVector.getY(), -poseEstimate.getX() + towerVector.getX());

                        drive.turnAsync(Angle.normDelta(targetAngle - poseEstimate.getHeading()) + 3.1415);

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    }

                    if(gamepad1.y)
                    {
                        resetPositionLine();

                        DcMotorEx finalOuttake = outtake;

                        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d(63.5, 0, 3.1415))
                                .strafeTo(new Vector2d(56.5, 41.2))
                                .addTemporalMarker(0.15, () -> {
                                    finalOuttake.setVelocity(POWERSHOT_VELO);
                                })
                                .build();

                        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                                .strafeTo(new Vector2d(56.5, 48.75))
                                .build();

                        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                                .strafeTo(new Vector2d(56.5, 56))
                                .build();

                        drive.followTrajectory(trajectory1);
                        out1.close();
                        out2.close();
                        sleep(250);
                        outg.cerc1();
                        sleep(250);
                        outg.open();
                        drive.followTrajectory(trajectory2);
                        outg.cerc2();
                        sleep(550);
                        outg.open();
                        drive.followTrajectory(trajectory3);
                        outg.close();
                        sleep(600);
                        outg.open();
                        sleep(250);
                        out1.open();
                        out2.open();
                    }

                    if (gamepad2.left_bumper && !cont_glisiera){
                        outtake.setVelocity(HIGH_VELO);
                        out1.close();
                        out2.close();
                        sleep(250);
                        outg.close();
                        cont_glisiera = true;
                    }
                    if (!gamepad2.left_bumper && cont_glisiera){
                        outg.open();
                        sleep(400);
                        out1.open();
                        out2.open();
                        cont_glisiera = false;
                        outtake.setVelocity(0);
                    }


                    if(gamepad2.x){
                        wob_brat.down();
                    }
                    else{
                        wob_brat.mid();
                    }
                    if(gamepad2.y){
                        wob_cleste.open();
                    }
                    else{
                        wob_cleste.close();
                    }
                    if(gamepad2.a){
                        plug.down();
                    }
                    if(!gamepad2.a){
                        plug.up();
                    }


                    if (gamepad2.dpad_up && !cont){
                        HIGH_VELO = HIGH_VELO + 20;
                        cont = !cont;
                    }
                    if (gamepad2.dpad_down && !contor){
                        HIGH_VELO = HIGH_VELO - 20;
                        contor = !contor;
                    }
                    if (!gamepad2.dpad_up){
                        cont = false;
                    }
                    if (!gamepad2.dpad_down){
                        contor = false;
                    }
                    if(gamepad2.dpad_left)
                    {
                        HIGH_VELO = 1380;
                    }
                    if(gamepad2.dpad_right)
                    {
                        HIGH_VELO = 1480;
                    }


                    if(gamepad2.right_trigger> 0) {
                        ok2 = !ok2;
                    }
                    else{
                        ok2 = false;
                    }

                    if(ok2){
                        outtake.setVelocity(-480);
                    }else{
                        outtake.setVelocity(Math.min(gamepad2.left_trigger*2000, HIGH_VELO));
                    }

                    if(gamepad2.right_bumper) {
                        ok3 = true;
                    }
                    else{
                        ok3 = false;
                    }

                    if(ok3){
                        intake.setPower(-0.8);
                    }else{
                        intake.setPower(Math.min(gamepad2.right_trigger, 0.9));
                    }

                    break;
                case AUTOMATIC_CONTROL:
                    // If x is pressed, we break out of the automatic following
                    if (gamepad1.x) {
                        drive.cancelFollowing();
                        currentMode = Mode.DRIVER_CONTROL;
                    }

                    // If drive finishes its task, cede control to the driver
                    if (!drive.isBusy()) {
                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    break;
            }
            telemetry.addData("outtake velocity", HIGH_VELO);
            telemetry.addData("mode", currentMode);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.update();

        }
    }
	
	void resetPositionCorner()
    {
        drive.setPoseEstimate(new Pose2d(0,0,0));
    }
    void resetPositionLine()
    {
        drive.setPoseEstimate(new Pose2d(63.5,0,3.1415));
    }

}

