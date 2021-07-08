package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.servo_glisiera;
import org.firstinspires.ftc.teamcode.hardware.servo_outtake1;
import org.firstinspires.ftc.teamcode.hardware.servo_outtake2;
import org.firstinspires.ftc.teamcode.hardware.servo_plug;
import org.firstinspires.ftc.teamcode.hardware.servo_wobble1;
import org.firstinspires.ftc.teamcode.hardware.servo_wobble2;


import android.graphics.ColorSpace;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;

@Autonomous
public class auto_remote extends LinearOpMode
{

    public static double NEW_P = 61;
    public static double NEW_I = 0.7;
    public static double NEW_D = 11;
    public static double NEW_F = 15.6;
    public double HIGH_VELO = 1480;
    public double POWERSHOT_VELO = 1180;

    public static double zero = 128;
    public static double unu = 136;
    public static double patru = 143;

    Boolean cont = Boolean.FALSE;
    Boolean contor = Boolean.FALSE;

    OpenCvCamera webcam;
    SkystoneDeterminationPipeline pipeline;

    @Override
    public void runOpMode()
    {

        Gamepad gp1 = gamepad1;

        int cameraMonitorViewId =
                hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        //phoneCam.setPipeline(pipeline);
        webcam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.

        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPSIDE_DOWN);
            }
        });



        servo_outtake1 out1 = new servo_outtake1(hardwareMap);
        servo_outtake2 out2 = new servo_outtake2(hardwareMap);
        servo_wobble1 wob_brat = new servo_wobble1(hardwareMap);
        servo_wobble2 wob_cleste = new servo_wobble2(hardwareMap);
        servo_glisiera outg = new servo_glisiera(hardwareMap);
        servo_plug plug = new servo_plug(hardwareMap);
        plug.up();
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

        PIDFCoefficients pidOrig = outtake.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // change coefficients using methods included with DcMotorEx class.
        PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        outtake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

        // re-read coefficients and verify change.
        PIDCoefficients pidModified = outtake.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        DcMotorEx finalOuttake = outtake;
        DcMotor finalIntake = intake;



        // *****************************  ZERO RINGS  ***************************** \\


        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                .strafeTo(new Vector2d(-55, -7.5))
                .addTemporalMarker(0.5, () -> {
                    finalOuttake.setVelocity(POWERSHOT_VELO);
                })
                .build();

        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .strafeTo(new Vector2d(-55, 3.25))
                .addTemporalMarker(0.1, () -> {
                    outg.open();
                })
                .build();

        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                .strafeTo(new Vector2d(-55, 11.5))
                .addTemporalMarker(0.1, () -> {
                    outg.open();
                })
                .build();

        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory3.end())
                .strafeTo(new Vector2d(-75.5, 36))
                .addTemporalMarker(0.1, () -> {
                    outg.open();
                    finalOuttake.setVelocity(0);
                })
                .addTemporalMarker(1.5, () -> {
                    wob_brat.down();
                    out1.open();
                    out2.open();
                })
                .build();


        Trajectory trajectory6 = drive.trajectoryBuilder(trajectory4.end())
                .splineToSplineHeading(new Pose2d(-22.25, 26, Math.toRadians(-45)), Math.toRadians(0))
                .build();


        Trajectory trajectory7 = drive.trajectoryBuilder(trajectory6.end(), true)
                .splineToSplineHeading(new Pose2d(-70, 26, Math.toRadians(0)), Math.toRadians(0))
                .build();

        Trajectory trajectory8 = drive.trajectoryBuilder(trajectory7.end(), true)
                .splineToSplineHeading(new Pose2d(-114.5, 39, Math.toRadians(-96)), Math.toRadians(0))
                .addTemporalMarker(0.3, () -> {
                    wob_cleste.close();
                    wob_cleste.open();
                    plug.down();
                    finalIntake.setPower(0.88);
                    finalOuttake.setVelocity(-500);
                })
                .build();

        Trajectory trajectory9 = drive.trajectoryBuilder(trajectory8.end())
                .strafeTo(new Vector2d(-119, -25))
                .build();

        Trajectory trajectory10 = drive.trajectoryBuilder(trajectory9.end(), true)
                .splineToSplineHeading(new Pose2d(-46, 22.5, Math.toRadians(0)), Math.toRadians(0))
                .addTemporalMarker(0.15, () -> {
                    plug.up();
                    finalIntake.setPower(0);
                    finalOuttake.setVelocity(0);
                    out1.close();
                    out2.close();
                })
                .addTemporalMarker(0.4, () -> {
                    finalOuttake.setVelocity(HIGH_VELO);
                })
                .build();

        Trajectory trajectory11 = drive.trajectoryBuilder(trajectory10.end())
                .strafeTo(new Vector2d(-67, 23))
                .build();





        // *****************************  ONE RING  ***************************** \\


        /*

        Trajectory trajectoryy1 = drive.trajectoryBuilder(new Pose2d())
                .strafeTo(new Vector2d(-53, -7.5))
                .addTemporalMarker(0.5, () -> {
                    finalOuttake.setVelocity(POWERSHOT_VELO);
                })
                .build();

        Trajectory trajectoryy2 = drive.trajectoryBuilder(trajectoryy1.end())
                .strafeTo(new Vector2d(-53, 3.25))
                .addTemporalMarker(0.1, () -> {
                    outg.open();
                })
                .build();

        Trajectory trajectoryy3 = drive.trajectoryBuilder(trajectoryy2.end())
                .strafeTo(new Vector2d(-53, 11.5))
                .addTemporalMarker(0.1, () -> {
                    outg.open();
                })
                .build();

         */

        Trajectory trajectoryy1 = drive.trajectoryBuilder(new Pose2d())
                .strafeTo(new Vector2d(-55, 0.5))
                .addTemporalMarker(0.5, () -> {
                    finalOuttake.setVelocity(POWERSHOT_VELO);
                })
                .build();

        Trajectory trajectoryy4 = drive.trajectoryBuilder(trajectoryy1.end().plus(new Pose2d(0, 0, Math.toRadians(10))), true)
                .splineToSplineHeading(new Pose2d(-113, 21, Math.toRadians(-93)), Math.toRadians(0))
                .addTemporalMarker(0.2, () -> {
                    outg.open();
                    finalOuttake.setVelocity(0);
                })
                .addTemporalMarker(1.8, () -> {
                    wob_brat.down();
                    out1.open();
                    out2.open();
                    plug.down();
                })
                .build();

        Trajectory trajectoryy44 = drive.trajectoryBuilder(trajectoryy4.end(), true)
                .splineToConstantHeading(new Vector2d(-113.5, -21), Math.toRadians(0))
                .addTemporalMarker(0.01, () -> {
                    finalOuttake.setVelocity(-500);
                    finalIntake.setPower(0.95);
                })
                .build();

        Trajectory trajectoryy5 = drive.trajectoryBuilder(trajectoryy4.end())
                /*
                .addTemporalMarker(0.6, () -> {
                    finalIntake.setPower(0.88);
                    finalOuttake.setVelocity(HIGH_VELO);
                })
                .addTemporalMarker(0.1, () -> {
                    out1.open();
                    out2.open();
                })

                 */

                .splineToConstantHeading(new Vector2d(-114, -19), Math.toRadians(0))
                .addTemporalMarker(0.01, () -> {
                    finalOuttake.setVelocity(-500);
                    finalIntake.setPower(0.95);
                })
                .splineToSplineHeading(new Pose2d(-51, 20.5, Math.toRadians(0)), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-28, 20.5), Math.toRadians(0),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(12, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory trajectoryy55 = drive.trajectoryBuilder(trajectoryy5.end())
                .strafeTo(new Vector2d(-22, 18))
                .build();


        Trajectory trajectoryy8 = drive.trajectoryBuilder(trajectoryy55.end())
                .strafeTo(new Vector2d(-89, 2))
                .build();

        Trajectory trajectoryy9 = drive.trajectoryBuilder(trajectoryy8.end())
                .strafeTo(new Vector2d(-69, 2))
                .build();





        // *****************************  FOUR RINGS  ***************************** \\


        Trajectory trajectoryyy1 = drive.trajectoryBuilder(new Pose2d())
                .strafeTo(new Vector2d(-55, 0.5))
                .addTemporalMarker(0.5, () -> {
                    finalOuttake.setVelocity(POWERSHOT_VELO);
                })
                .build();


        Trajectory trajectoryyy2 = drive.trajectoryBuilder(trajectoryyy1.end().plus(new Pose2d(0, 0, Math.toRadians(17.5))), true)
                .splineToSplineHeading(new Pose2d(-104, 34.5, Math.toRadians(30)), Math.toRadians(0))
                .addTemporalMarker(1.6, () -> {
                    wob_brat.down();
                    outg.open();
                    finalOuttake.setVelocity(0);
                })
                .build();

        Trajectory trajectoryyy3 = drive.trajectoryBuilder(trajectoryyy2.end())
                .splineToSplineHeading(new Pose2d(-52, 19, Math.toRadians(0)), Math.toRadians(-30))
                .splineToConstantHeading(new Vector2d(-28, 19), Math.toRadians(0),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(12, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.32, () -> {
                    plug.down();
                    wob_cleste.close();
                    out1.open();
                    out2.open();
                    finalOuttake.setVelocity(-450);
                    finalIntake.setPower(0.95);
                })
                .build();

        Trajectory trajectoryyy4 = drive.trajectoryBuilder(trajectoryyy3.end().plus(new Pose2d(0, 0, Math.toRadians(5))))
                .splineToConstantHeading(new Vector2d(-15, 19), Math.toRadians(0),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(11, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(new Vector2d(-15, 21), Math.toRadians(0))
                .addTemporalMarker(0.01, () -> {
                    finalIntake.setPower(0.95);
                    finalOuttake.setVelocity(-200);
                })
                .addTemporalMarker(1.9, () -> {
                    finalIntake.setPower(0);
                    out1.close();
                    out2.close();
                })
                .build();

        /*

        Trajectory trajectoryyy5 = drive.trajectoryBuilder(trajectoryyy4.end())
                .strafeTo(new Vector2d(-15, 21))
                .addTemporalMarker(0.1, () -> {
                    out1.close();
                    out2.close();
                    finalIntake.setPower(0);
                })
                .build();

         */

        Trajectory trajectoryyy6 = drive.trajectoryBuilder(trajectoryyy4.end())
                .strafeTo(new Vector2d(-50, 30))
                .addTemporalMarker(0.02, () -> {
                    finalOuttake.setVelocity(HIGH_VELO);
                })
                .build();

        Trajectory trajectoryyy7 = drive.trajectoryBuilder(trajectoryyy6.end())
                .strafeTo(new Vector2d(-115, 30))
                .addTemporalMarker(0.1, () -> {
                    finalOuttake.setVelocity(0);
                    outg.open();
                })
                .addTemporalMarker(1.9, () -> {
                    out1.open();
                    out2.open();
                    wob_cleste.open();
                })
                .build();

        Trajectory trajectoryyy8 = drive.trajectoryBuilder(trajectoryyy7.end())
                .strafeTo(new Vector2d(-77, 30))
                .build();




        while(!isStarted())
        {
            telemetry.addData("Analysis", pipeline.avg1);
            telemetry.addData("Position", pipeline.position);
            telemetry.addData("press x for 0 -", zero);
            telemetry.addData("press y for 1 -", unu);
            telemetry.addData("press a for 4 -", patru);
            telemetry.update();

            if(gp1.x)
            {
                while(gp1.b == false)
                {
                    telemetry.addData("value for 0 -", zero);
                    telemetry.addData("press dpad up to increase by", 1);
                    telemetry.addData("press dpad down to decrease by", 1);
                    telemetry.addData("press b to save and exit", "");
                    telemetry.update();

                    if (gp1.dpad_up && !cont){
                        zero = zero + 1;
                        cont = !cont;
                    }
                    if (gp1.dpad_down && !contor){
                        zero = zero - 1;
                        contor = !contor;
                    }
                    if (!gp1.dpad_up){
                        cont = false;
                    }
                    if (!gp1.dpad_down){
                        contor = false;
                    }
                }
            }

            if(gp1.y)
            {
                while(gp1.b == false)
                {
                    telemetry.addData("value for 1 -", unu);
                    telemetry.addData("press dpad up to increase by", 1);
                    telemetry.addData("press dpad down to decrease by", 1);
                    telemetry.addData("press b to save and exit", "");
                    telemetry.update();

                    if (gp1.dpad_up && !cont){
                        unu = unu + 1;
                        cont = !cont;
                    }
                    if (gp1.dpad_down && !contor){
                        unu = unu - 1;
                        contor = !contor;
                    }
                    if (!gp1.dpad_up){
                        cont = false;
                    }
                    if (!gp1.dpad_down){
                        contor = false;
                    }
                }
            }

            if(gp1.a)
            {
                while(gp1.b == false)
                {
                    telemetry.addData("value for 4 -", patru);
                    telemetry.addData("press dpad up to increase by", 1);
                    telemetry.addData("press dpad down to decrease by", 1);
                    telemetry.addData("press b to save and exit", "");
                    telemetry.update();

                    if (gp1.dpad_up && !cont){
                        patru = patru + 1;
                        cont = !cont;
                    }
                    if (gp1.dpad_down && !contor){
                        patru = patru - 1;
                        contor = !contor;
                    }
                    if (!gp1.dpad_up){
                        cont = false;
                    }
                    if (!gp1.dpad_down){
                        contor = false;
                    }
                }
            }
        }

        waitForStart();


        while (opModeIsActive())
        {
            out1.close();
            out2.close();
            wob_brat.mid();
            if(pipeline.zona == 0)
            {
                drive.followTrajectory(trajectory1);
                outg.cerc1();
                sleep(250);
                drive.followTrajectory(trajectory2);
                outg.cerc2();
                sleep(450);
                drive.followTrajectory(trajectory3);
                outg.close();
                sleep(700);

                drive.followTrajectory(trajectory4);
                wob_cleste.setServoPositions(0.99);
                sleep(150);
                drive.followTrajectory(trajectory6);
                wob_cleste.close();
                sleep(340);
                drive.followTrajectory(trajectory7);
                wob_cleste.open();
                sleep(150);
                wob_brat.mid();

                drive.followTrajectory(trajectory8);
                drive.followTrajectory(trajectory9);
                drive.followTrajectory(trajectory10);
                outg.close();
                sleep(350);
                drive.followTrajectory(trajectory11);
                outg.open();
            }

            else if(pipeline.zona == 1)
            {
                drive.followTrajectory(trajectoryy1);
                /*
                outg.cerc1();
                sleep(250);
                drive.followTrajectory(trajectoryy2);
                outg.cerc2();
                sleep(500);
                drive.followTrajectory(trajectoryy3);
                outg.close();
                sleep(600);

                 */

                outg.cerc1();
                sleep(330);
                drive.turn(Math.toRadians(-10.5));
                outg.cerc2();
                sleep(330);
                drive.turn(Math.toRadians(18.75));
                outg.close();
                sleep(330);


                drive.followTrajectory(trajectoryy4);
                wob_cleste.open();
                sleep(200);
                drive.followTrajectory(trajectoryy5);
                outtake.setVelocity(HIGH_VELO-100);
                intake.setPower(0);
                out1.close();
                out2.close();
                sleep(750);
                outg.close();
                sleep(250);
                outg.open();
                sleep(100);
                outg.close();
                sleep(750);


                drive.followTrajectory(trajectoryy55);
                wob_cleste.close();
                outg.close();
                outtake.setVelocity(0);
                sleep(400);
                drive.followTrajectory(trajectoryy8);
                wob_cleste.open();
                sleep(350);
                wob_brat.up();
                drive.followTrajectory(trajectoryy9);
            }

            else if(pipeline.zona == 4)
            {
                drive.followTrajectory(trajectoryyy1);
                outg.cerc1();
                sleep(330);
                drive.turn(Math.toRadians(-10.5));
                outg.cerc2();
                sleep(330);
                drive.turn(Math.toRadians(20));
                outg.close();
                sleep(330);

                drive.followTrajectory(trajectoryyy2);
                wob_cleste.open();
                drive.followTrajectory(trajectoryyy3);
                out1.close();
                out2.close();
                intake.setPower(0);
                outtake.setVelocity(HIGH_VELO);
                drive.turn(Math.toRadians(-5));
                sleep(1200);
                outg.close();
                sleep(800);
                outtake.setVelocity(0);
                outg.open();
                wob_brat.down();
                wob_cleste.open();
                sleep(250);
                out1.open();
                out2.open();

                drive.followTrajectory(trajectoryyy4);
                //drive.followTrajectory(trajectoryyy5);
                wob_cleste.close();
                sleep(250);
                drive.followTrajectory(trajectoryyy6);
                outg.close();
                sleep(800);
                drive.followTrajectory(trajectoryyy7);
                drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                drive.followTrajectory(trajectoryyy8);
            }

            out1.close();
            out2.close();
            stop();

        }


    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        public double zona = 4;

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(260,50);

        static final int REGION_WIDTH = 35;
        static final int REGION_HEIGHT = 30;

        final int FOUR_RING_THRESHOLD = (int) Math.round((patru+unu)/2);
        final int ONE_RING_THRESHOLD = (int) Math.round((zero+unu)/2);

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingPosition position = RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    -5); // Thickness of the rectangle lines

            position = RingPosition.ONE; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = RingPosition.FOUR;
                zona = 4;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = RingPosition.ONE;
                zona = 1;
            }else{
                position = RingPosition.NONE;
                zona = 0;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -5); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }
}