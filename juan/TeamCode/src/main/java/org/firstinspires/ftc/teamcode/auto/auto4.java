package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.hardware.init_robot;
import org.firstinspires.ftc.teamcode.hardware.servo_outtake1;
import org.firstinspires.ftc.teamcode.hardware.servo_outtake2;
import org.firstinspires.ftc.teamcode.hardware.servo_wobble1;
import org.firstinspires.ftc.teamcode.hardware.servo_wobble2;
import org.firstinspires.ftc.teamcode.hardware.servo_glisiera;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.sun.tools.javac.util.Position;

import java.util.Arrays;
import java.util.List;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000 to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous
@Disabled
public class auto4 extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static double zona = 0.0;

    private static final String VUFORIA_KEY =
            "AQvxOqn/////AAABmQw9oeoIkUcTh/YoGrqERoaCnanGje5GD769dIaZMZMtr1fEifi7NCszU8vqMoDWN/iKKTRSQX2MeQwXuyR3l68zLtFHTYSebT/dy/nA8+HcfHt/YDIQzkrXzhzusx58LFfj0Drl0oh/u2YC01rCVAkV9U6Uzn4X3gJagvqzEynqHxC9DtWuXxXkltT59e7tVxToKoZvD9ohB+xz+wd8wXS+qeOBRlwruqbtR9uAZqHgUUq3ruC/79mRRfjPaGE83UhYYwTeRlDwaLVVO6H/boJTfV29ReNrdxQf18scqJJKUqKkImV97TY5dmadU9fpRFENZjAE4r7XZcJ+Sc/pXhlQGQ5PhvkkBVvEpk+xPblY";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    DcMotorEx outtake;

    public static double NEW_P = 38;
    public static double NEW_I = 15;
    public static double NEW_D = 20;
    public static double HIGH_VELO = 1475;
    public static double POWERSHOT_VELO = 1300;
    public static double unghi = 15.5;

    @Override
    public void runOpMode() {

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.

        servo_outtake1 out1 = new servo_outtake1(hardwareMap);
        servo_outtake2 out2 = new servo_outtake2(hardwareMap);
        servo_wobble1 wob_brat = new servo_wobble1(hardwareMap);
        servo_wobble2 wob_cleste = new servo_wobble2(hardwareMap);
        servo_glisiera outg = new servo_glisiera(hardwareMap);
        out1.close();
        out2.close();
        outg.open();
        wob_brat.up();
        wob_cleste.close();


        DcMotor intake = null; // Intake motor
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setPower(0.0);



        DcMotorEx outtake = null; // Intake motor
        outtake = (DcMotorEx)hardwareMap.get(DcMotor.class, "outtake");
        PIDCoefficients pidOrig = outtake.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDCoefficients pidNew = new PIDCoefficients(NEW_P, NEW_I, NEW_D);
        outtake.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        PIDCoefficients pidModified = outtake.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);



        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        DcMotorEx finalOuttake = outtake;
        DcMotor finalIntake = intake;


        DcMotorEx finalOuttake1 = outtake;
        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                .strafeTo(new Vector2d(-55, -4))
                .addTemporalMarker(0.5, () -> {
                    finalOuttake1.setVelocity(HIGH_VELO);
                })
                .build();

        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end().plus(new Pose2d(0, 0, Math.toRadians(unghi))))
                .strafeTo(new Vector2d(-68, -4))
                .addTemporalMarker(0.2, () -> {
                    finalOuttake1.setVelocity(0);
                })
                .build();

        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                .lineToLinearHeading(new Pose2d(-24, 3, Math.toRadians(0)))
                .build();

        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory3.end())
                .strafeTo(new Vector2d(-68, -21.5),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(14, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();






        Trajectory trajectoryy1 = drive.trajectoryBuilder(new Pose2d())
                .strafeTo(new Vector2d(-55, -4))
                .addTemporalMarker(0.5, () -> {
                    finalOuttake1.setVelocity(HIGH_VELO);
                })
                .build();

        Trajectory trajectoryy2 = drive.trajectoryBuilder(trajectoryy1.end().plus(new Pose2d(0, 0, Math.toRadians(unghi))))
                .lineToLinearHeading(new Pose2d(-92, -2, Math.toRadians(180)))
                .addTemporalMarker(0.2, () -> {
                    finalOuttake1.setVelocity(0);
                })
                .build();

        Trajectory trajectoryy3 = drive.trajectoryBuilder(trajectoryy2.end())
                .lineToLinearHeading(new Pose2d(-22, 5, Math.toRadians(0)))
                .build();

        Trajectory trajectoryy4 = drive.trajectoryBuilder(trajectoryy3.end())
                .strafeTo(new Vector2d(-70, 4))
                .build();




        Trajectory trajectoryyy1 = drive.trajectoryBuilder(new Pose2d())
                .strafeTo(new Vector2d(-55, -4))
                .addTemporalMarker(0.5, () -> {
                    finalOuttake1.setVelocity(HIGH_VELO);
                })
                .build();

        Trajectory trajectoryyy2 = drive.trajectoryBuilder(trajectoryyy1.end().plus(new Pose2d(0, 0, Math.toRadians(unghi))))
                .lineToLinearHeading(new Pose2d(-109, -2, Math.toRadians(25)))
                .addTemporalMarker(0.2, () -> {
                    finalOuttake1.setVelocity(0);
                })
                .build();

        Trajectory trajectoryyy3 = drive.trajectoryBuilder(trajectoryyy2.end())
                .lineToLinearHeading(new Pose2d(-22, 5, Math.toRadians(0)))
                .build();

        Trajectory trajectoryyy4 = drive.trajectoryBuilder(trajectoryyy3.end())
                .strafeTo(new Vector2d(-70, 4))
                .build();

        /*
        Trajectory trajectory5 = drive.trajectoryBuilder(trajectory4.end())
                .strafeTo(new Vector2d(-79.5, -9.5))
                .build();

        Trajectory trajectory6 = drive.trajectoryBuilder(trajectory5.end())
                .strafeTo(new Vector2d(-77, -28))
                .build();




        Trajectory trajectoryy1 = drive.trajectoryBuilder(new Pose2d())
                .back(45)
                .splineToConstantHeading(new Vector2d(-99, -28), Math.toRadians(0))
                .addTemporalMarker(3.95, () -> {
                    wob1.close();
                })
                .build();

        Trajectory trajectoryy2 = drive.trajectoryBuilder(trajectoryy1.end())
                .strafeTo(new Vector2d(-55, -10))
                .build();

        Trajectory trajectoryy201 = drive.trajectoryBuilder(trajectoryy2.end())
                .strafeTo(new Vector2d(-55, -22))
                .build();

        Trajectory trajectoryy21 = drive.trajectoryBuilder(trajectoryy201.end())
                .strafeTo(new Vector2d(-40, -20.5),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                .addTemporalMarker(0.1, () -> {
                    out1.open();
                    out2.open();
                    finalIntake.setPower(0.88);
                })
                .build();

        Trajectory trajectoryy22 = drive.trajectoryBuilder(trajectoryy21.end())
                .strafeTo(new Vector2d(-55, -10))
                .addTemporalMarker(0.15, () -> {
                    finalIntake.setPower(0);
                    finalOuttake.setVelocity(HIGH_VELO);
                    wob2.close();
                })
                .build();

        Trajectory trajectoryy3 = drive.trajectoryBuilder(trajectoryy22.end())
                .strafeTo(new Vector2d(-27, -64))
                .addTemporalMarker(2.4, () -> {
                    wob2.open();
                })
                .build();

        Trajectory trajectoryy4 = drive.trajectoryBuilder(trajectoryy3.end())
                .strafeTo(new Vector2d(-20.2, -48))
                .build();

        Trajectory trajectoryy5 = drive.trajectoryBuilder(trajectoryy4.end())
                .strafeTo(new Vector2d(-101, -35))
                .build();

        Trajectory trajectoryy6 = drive.trajectoryBuilder(trajectoryy5.end())
                .strafeTo(new Vector2d(-78, -40))
                .build();








        Trajectory trajectoryyy1 = drive.trajectoryBuilder(new Pose2d())
                .back(115)
                .splineToConstantHeading(new Vector2d(-55, -20),Math.toRadians(0))
                .addTemporalMarker(3.2, () -> {
                    wob1.setServoPositions(0.65);
                })
                .addTemporalMarker(3.45, () -> {
                    wob2.open();
                })
                .addTemporalMarker(3.8, () -> {
                    wob1.close();
                    finalOuttake.setVelocity(POWERSHOT_VELO);
                })
                .build();


        Trajectory trajectoryyy3 = drive.trajectoryBuilder(trajectoryyy1.end())
                .strafeTo(new Vector2d(-55, -44),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(15, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.4, () -> {
                    out3.close();
                })
                .build();

        Trajectory trajectoryyy4 = drive.trajectoryBuilder(trajectoryyy3.end())
                .strafeTo(new Vector2d(-20.5, -44))
                .addTemporalMarker(0.1, () -> {
                    finalOuttake.setVelocity(0);
                    out3.open();
                })
                .build();

        Trajectory trajectoryyy5 = drive.trajectoryBuilder(trajectoryyy4.end())
                .strafeTo(new Vector2d(-20.5, -39.75))
                .addTemporalMarker(0.2, () -> {
                    out1.open();
                    out2.open();
                })
                .build();

        Trajectory trajectoryyy6 = drive.trajectoryBuilder(trajectoryyy5.end())
                .splineToConstantHeading(new Vector2d(-120, -6.5), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-44, -18.75), Math.toRadians(0))
                .addTemporalMarker(3.3, () -> {
                    wob1.setServoPositions(0.75);
                })
                .addTemporalMarker(3, () -> {
                    wob2.open();
                })
                .build();



        Trajectory trajectoryyy8 = drive.trajectoryBuilder(trajectoryyy6.end())
                .strafeTo(new Vector2d(-37, -18),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(12, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.05, () -> {
                    finalIntake.setPower(-0.85);
                })
                .build();

        Trajectory trajectoryyy9 = drive.trajectoryBuilder(trajectoryyy8.end())
                .strafeTo(new Vector2d(-32, -18),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(5, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.00, () -> {
                    finalIntake.setPower(0.85);
                })
                .build();

        Trajectory trajectoryyy10 = drive.trajectoryBuilder(trajectoryyy9.end())
                .strafeTo(new Vector2d(-57, -12),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(12, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.0, () -> {
                    finalOuttake.setVelocity(HIGH_VELO);
                })
                .addTemporalMarker(0.2, () -> {
                    out1.close();
                    out2.close();
                })
                .addTemporalMarker(1.1, () -> {
                    out3.close();
                })
                .build();

        Trajectory trajectoryyy11 = drive.trajectoryBuilder(trajectoryyy10.end())
                .strafeTo(new Vector2d(-28, -18))
                .addTemporalMarker(0.5, () -> {
                    out1.open();
                    out2.open();
                })
                .addTemporalMarker(0.05, () -> {
                    out3.open();
                })
                .build();

        Trajectory trajectoryyy12 = drive.trajectoryBuilder(trajectoryyy11.end())
                .strafeTo(new Vector2d(-72, -12),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(9, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.3, () -> {
                    finalIntake.setPower(0);
                    out1.close();
                    out2.close();
                })
                .addTemporalMarker(1, () -> {
                    out3.close();
                })
                .build();










        Trajectory trajectoryyyy1 = drive.trajectoryBuilder(new Pose2d())
                .strafeTo(new Vector2d(-120, -6))
                .addTemporalMarker(3.55, () -> {
                    wob1.setServoPositions(0.64);
                })
                .addTemporalMarker(3.85, () -> {
                    wob2.open();
                })
                .build();

        Trajectory trajectoryyyy2 = drive.trajectoryBuilder(trajectoryyyy1.end())
                .strafeTo(new Vector2d(-55, -8))
                .addTemporalMarker(0.1, () -> {
                    finalOuttake.setVelocity(HIGH_VELO);
                })
                .addTemporalMarker(0.6, () -> {
                    wob1.close();
                })
                .build();
        Trajectory trajectoryyyy3 = drive.trajectoryBuilder(trajectoryyyy2.end())
                .strafeTo(new Vector2d(-56, -16))
                .addTemporalMarker(0.4, () -> {
                    finalOuttake.setVelocity(-150);
                    finalIntake.setPower(0.87);
                    out1.open();
                    out2.open();
                })
                .build();

        Trajectory trajectoryyyy4 = drive.trajectoryBuilder(trajectoryyyy3.end())
                .strafeTo(new Vector2d(-46, -16))
                .build();

        Trajectory trajectoryyyy40 = drive.trajectoryBuilder(trajectoryyyy4.end())
                .strafeTo(new Vector2d(-39, -16),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(7, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0, () -> {
                    finalIntake.setPower(0.87);
                    finalOuttake.setVelocity(-150);
                })
                .build();
        Trajectory trajectoryyyy5 = drive.trajectoryBuilder(trajectoryyyy40.end())
                .strafeTo(new Vector2d(-55, -7.5))
                .addTemporalMarker(0.3, () -> {
                    finalOuttake.setVelocity(HIGH_VELO);
                    finalIntake.setPower(0.0);
                    out1.close();
                    out2.close();
                })
                .build();

        Trajectory trajectoryyyy6 = drive.trajectoryBuilder(trajectoryyyy5.end())
                .strafeTo(new Vector2d(-40, -17))
                .addTemporalMarker(0.4, () -> {
                    finalOuttake.setVelocity(-150);
                    finalIntake.setPower(0.87);
                    out1.open();
                    out2.open();
                })
                .build();
        Trajectory trajectoryyyy7 = drive.trajectoryBuilder(trajectoryyyy6.end())
                .strafeTo(new Vector2d(-25.5, -17),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(7, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory trajectoryyyy8 = drive.trajectoryBuilder(trajectoryyyy7.end())
                .strafeTo(new Vector2d(-55, -7.5)                )
                .addTemporalMarker(0.2, () -> {
                    finalOuttake.setVelocity(HIGH_VELO);
                    finalIntake.setPower(0);
                    out1.close();
                    out2.close();
                })
                .build();

        Trajectory trajectoryyyy9 = drive.trajectoryBuilder(trajectoryyyy8.end())
                .strafeTo(new Vector2d(-70, -7.5)                )
                .addTemporalMarker(0.02, () -> {
                    out3.open();
                })
                .build();

         */


        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(4.0, 16.0/9.0);
        }

        /** Wait for the game to begin */

        while(!isStarted())
        {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    //telemetry.addData("# Object Detected", updatedRecognitions.size());
                    telemetry.update();
                    if (updatedRecognitions.size() == 0) {
                        // empty list.  no objects recognized.
                        telemetry.addData("0", "");
                        zona = 0;
                    } else {
                        // list is not empty.
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {

                            // check label to see which target zone to go after.
                            if (recognition.getLabel().equals("Single")) {
                                zona = 1.0;
                                telemetry.addData("1", "");
                            } else if (recognition.getLabel().equals("Quad")) {
                                zona = 4.0;
                                telemetry.addData("4", "");
                            } else {
                                zona = 0.0;
                            }
                        }
                    }
                }
            }
        }


        waitForStart();

        if (tfod != null) {
            tfod.shutdown();
            telemetry.addData("Vision stopped", "");
            telemetry.addData("Rings ", zona);
            telemetry.update();
        }

        if (isStopRequested()) return;

        Pose2d poseEstimate = drive.getPoseEstimate();

        while (!isStopRequested() && opModeIsActive())
        {

            if(zona == 0.0)
            {

                telemetry.addData("Zero ringuri", "");
                telemetry.update();

                drive.followTrajectory(trajectory1);
                drive.turn(Math.toRadians(unghi));
                sleep(250);
                outg.close();
                sleep(1200);
                outg.open();
                drive.followTrajectory(trajectory2);
                wob_brat.down();
                sleep(750);
                wob_cleste.open();
                sleep(500);
                wob_cleste.close();
                wob_brat.mid();
                drive.followTrajectory(trajectory3);
                out1.open();
                out2.open();
                sleep(5000);
                drive.followTrajectory(trajectory4);

                /*
                drive.followTrajectory(trajectory1);
                //drive.turn(Math.toRadians(-4));
                wob2.open();
                outtake.setVelocity(HIGH_VELO);
                drive.followTrajectory(trajectory2);
                sleep(200);
                out3.close();
                sleep(1250);
                out3.open();
                outtake.setVelocity(0);
                drive.followTrajectory(trajectory3);
                out1.open();
                out2.open();
                drive.followTrajectory(trajectory4);
                wob2.close();
                sleep(300);

                drive.followTrajectory(trajectory5);
                wob2.open();
                drive.followTrajectory(trajectory6);
                drive.turn(Math.toRadians(186));
                 */
            }


            if(zona == 1.0)
            {
                telemetry.addData("1 ring", "");
                telemetry.update();


                drive.followTrajectory(trajectoryy1);
                drive.turn(Math.toRadians(unghi));
                sleep(250);
                outg.close();
                sleep(1200);
                outg.open();
                drive.followTrajectory(trajectoryy2);
                wob_brat.down();
                sleep(750);
                wob_cleste.open();
                sleep(500);
                wob_cleste.close();
                wob_brat.mid();
                drive.followTrajectory(trajectoryy3);
                out1.open();
                out2.open();
                sleep(5000);
                drive.followTrajectory(trajectoryy4);

                /*
                drive.followTrajectory(trajectoryy1);
                //drive.turn(Math.toRadians(-4));
                wob2.open();
                outtake.setVelocity(HIGH_VELO);
                drive.followTrajectory(trajectoryy2);
                //drive.turn(Math.toRadians(-7));
                sleep(200);
                out3.close();
                sleep(1500);
                out3.open();
                outtake.setVelocity(0);
                sleep(400);

                drive.followTrajectory(trajectoryy21);
                out1.close();
                out2.close();
                drive.followTrajectory(trajectoryy22);
                //drive.turn(Math.toRadians(-6));

                sleep(200);
                out3.close();
                sleep(1200);
                out3.open();
                outtake.setVelocity(0);

                drive.followTrajectory(trajectoryy3);
                out1.open();
                out2.open();
                drive.followTrajectory(trajectoryy4);
                wob2.close();
                sleep(400);

                drive.followTrajectory(trajectoryy5);
                wob2.open();
                drive.followTrajectory(trajectoryy6);
                drive.turn(Math.toRadians(186));

                 */

            }

            if(zona == 4.0)
            {
                telemetry.addData("4 ringuri", "");
                telemetry.update();


                drive.followTrajectory(trajectoryyy1);
                drive.turn(Math.toRadians(unghi));
                sleep(250);
                outg.close();
                sleep(1200);
                outg.open();
                drive.followTrajectory(trajectoryyy2);
                wob_brat.down();
                sleep(750);
                wob_cleste.open();
                sleep(500);
                wob_cleste.close();
                wob_brat.mid();
                drive.followTrajectory(trajectoryyy3);
                out1.open();
                out2.open();
                sleep(12000);
                drive.followTrajectory(trajectoryyy4);

                /*
                drive.followTrajectory(trajectoryyy1);
                //wob2.open();
                //drive.followTrajectory(trajectoryyy2);
                //drive.turn(Math.toRadians(-5));
                out3.cerc1();
                sleep(150);
                drive.followTrajectory(trajectoryyy3);
                drive.followTrajectory(trajectoryyy4);
                drive.followTrajectory(trajectoryyy5);
                wob2.close();
                sleep(650);
                wob1.setServoPositions(0.64);
                sleep(350);
                drive.followTrajectory(trajectoryyy6);
                //wob2.open();
                //drive.followTrajectory(trajectoryyy7);
                drive.followTrajectory(trajectoryyy8);
                intake.setPower(0);
                drive.followTrajectory(trajectoryyy9);
                out1.close();
                out2.close();
                //outtake.setVelocity(HIGH_VELO);
                drive.followTrajectory(trajectoryyy10);
                //out3.cerc2();
                //sleep(700);
                //out3.open();
                //outtake.setVelocity(0);
                drive.followTrajectory(trajectoryyy11);
                drive.followTrajectory(trajectoryyy12);
                //out3.close();
                //sleep(800);
                //out3.open();
                //drive.followTrajectory(trajectoryyy13);



                drive.followTrajectory(trajectoryyyy1);
                drive.followTrajectory(trajectoryyyy2);
                drive.turn(Math.toRadians(-4));
                out3.close();
                sleep(1400);
                out3.open();
                //drive.turn(Math.toRadians(-2));
                drive.followTrajectory(trajectoryyyy3);
                //drive.turn(Math.toRadians(-6));
                drive.followTrajectory(trajectoryyyy4);
                drive.followTrajectory(trajectoryyyy40);
                drive.followTrajectory(trajectoryyyy5);
                drive.turn(Math.toRadians(-3));
                sleep(150);
                out3.close();
                sleep(1400);
                out3.open();
                drive.followTrajectory(trajectoryyyy6);
                drive.followTrajectory(trajectoryyyy7);
                //drive.turn(Math.toRadians(-19));
                drive.followTrajectory(trajectoryyyy8);
                out3.close();
                sleep(1250);
                out3.open();
                drive.followTrajectory(trajectoryyyy9);

                 */
            }

            stop();
        };

    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}