package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.servo_glisiera;
import org.firstinspires.ftc.teamcode.hardware.servo_intake;
import org.firstinspires.ftc.teamcode.hardware.servo_outtake1;
import org.firstinspires.ftc.teamcode.hardware.servo_outtake2;
import org.firstinspires.ftc.teamcode.hardware.servo_plug;
import org.firstinspires.ftc.teamcode.hardware.servo_wobble1;
import org.firstinspires.ftc.teamcode.hardware.servo_wobble2;
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
//@Disabled
public class red_wall extends LinearOpMode
{

    public static double HIGH_VELO = 1540;

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
        servo_intake serv_int = new servo_intake(hardwareMap);
        serv_int.down();
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


        DcMotorEx motorOuttake1 = hardwareMap.get(DcMotorEx.class, "outtake1");
        DcMotorEx motorOuttake2 = hardwareMap.get(DcMotorEx.class, "outtake2");

        motorOuttake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorOuttake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        DcMotor finalIntake = intake;



        // *****************************  ZERO RINGS  ***************************** \\


        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d(), true)
                .splineTo(new Vector2d(-58, 1.5), Math.toRadians(195))
                .addTemporalMarker(1.2, () -> {
                    motorOuttake1.setVelocity(HIGH_VELO);
                    motorOuttake2.setVelocity(HIGH_VELO);
                })
                .build();

        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end(), true)
                .splineTo(new Vector2d(-71.5, -6.5), Math.toRadians(200))
                .addTemporalMarker(0.05, () -> {
                    outg.open();
                    motorOuttake1.setVelocity(0);
                    motorOuttake2.setVelocity(0);
                })
                .addTemporalMarker(0.5, () -> {
                    wob_brat.down();
                })
                .build();

        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                .splineTo(new Vector2d(-10, 0), Math.toRadians(0))
                .addTemporalMarker(0.1, () -> {
                    wob_brat.up();
                    wob_cleste.close();
                })
                .build();

        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory3.end(), true)
                .splineTo(new Vector2d(-73.5, -15), Math.toRadians(178))
                .addTemporalMarker(0.25, () -> {
                    plug.down();
                })
                .addTemporalMarker(0.55, () -> {
                    serv_int.up();
                })
                .addTemporalMarker(0.9, () -> {
                    plug.up();
                })
                .build();





        // *****************************  ONE RING  ***************************** \\


        Trajectory trajectoryy1 = drive.trajectoryBuilder(new Pose2d(), true)
                .splineTo(new Vector2d(-58, 1.5), Math.toRadians(195))
                .addTemporalMarker(1.2, () -> {
                    motorOuttake1.setVelocity(HIGH_VELO);
                    motorOuttake2.setVelocity(HIGH_VELO);
                })
                .build();

        Trajectory trajectoryy2 = drive.trajectoryBuilder(trajectoryy1.end(), true)
                .splineTo(new Vector2d(-92.5, 4), Math.toRadians(0))
                .addTemporalMarker(0.1, () -> {
                    outg.open();
                    motorOuttake1.setVelocity(0);
                    motorOuttake2.setVelocity(0);
                })
                .addTemporalMarker(0.4, () -> {
                    wob_brat.up();
                })
                .build();

        Trajectory trajectoryy3 = drive.trajectoryBuilder(trajectoryy2.end())
                .splineToConstantHeading(new Vector2d(-72, 0), Math.toRadians(0))
                .splineTo(new Vector2d(-62, -17.5), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-38, -17.5), Math.toRadians(0),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(11, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.1, () -> {
                    wob_brat.up();
                    wob_cleste.close();
                    out1.open();
                    out2.open();
                })
                .addTemporalMarker(0.8, () -> {
                    plug.down();
                })
                .addTemporalMarker(1.2, () -> {
                    serv_int.up();
                })
                .addTemporalMarker(1.5, () -> {
                    plug.up();
                    finalIntake.setPower(0.95);
                })
                .build();

        Trajectory trajectoryy4 = drive.trajectoryBuilder(trajectoryy3.end(), true)
                //.splineToConstantHeading(new Vector2d(-52, -17.5), Math.toRadians(0))
                .lineToSplineHeading(new Pose2d(-54, -4.5, Math.toRadians(7)))
                .addTemporalMarker(0.02, () -> {
                    out1.close();
                    out2.close();
                    finalIntake.setPower(0);
                })
                .addTemporalMarker(0.4, () -> {
                    motorOuttake1.setVelocity(HIGH_VELO);
                    motorOuttake2.setVelocity(HIGH_VELO);
                })
                .build();

        Trajectory trajectoryy5 = drive.trajectoryBuilder(trajectoryy4.end(), true)
                .strafeTo(new Vector2d(-73, 1))
                .addTemporalMarker(0.1, () -> {
                    motorOuttake1.setVelocity(0);
                    motorOuttake2.setVelocity(0);
                })
                .build();






        // *****************************  FOUR RINGS  ***************************** \\


        Trajectory trajectoryyy1 = drive.trajectoryBuilder(new Pose2d(), true)
                .splineTo(new Vector2d(-58, 1.5), Math.toRadians(195))
                .addTemporalMarker(1.2, () -> {
                    motorOuttake1.setVelocity(HIGH_VELO);
                    motorOuttake2.setVelocity(HIGH_VELO);
                })
                .build();

        Trajectory trajectoryyy2 = drive.trajectoryBuilder(trajectoryyy1.end(), true)
                .splineTo(new Vector2d(-119.5, -6.5), Math.toRadians(180))
                .addTemporalMarker(0.1, () -> {
                    outg.open();
                    motorOuttake1.setVelocity(0);
                    motorOuttake2.setVelocity(0);
                })
                .addTemporalMarker(1.75, () -> {
                    wob_brat.down();
                })
                .build();

        Trajectory trajectoryyy3 = drive.trajectoryBuilder(trajectoryyy2.end())
                .splineToConstantHeading(new Vector2d(-72, 0), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-62, -19), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-40, -19), Math.toRadians(0),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(11, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.05, () -> {
                    wob_brat.up();
                    wob_cleste.close();
                    out1.open();
                    out2.open();
                })
                .addTemporalMarker(1.05, () -> {
                    plug.down();
                })
                .addTemporalMarker(1.55, () -> {
                    serv_int.up();
                })
                .addTemporalMarker(2.2, () -> {
                    finalIntake.setPower(0.95);
                })
                .build();

        Trajectory trajectoryyy4 = drive.trajectoryBuilder(trajectoryyy3.end())
                .splineToConstantHeading(new Vector2d(-52, -18), Math.toRadians(0))
                .addTemporalMarker(0.02, () -> {
                    out1.close();
                    out2.close();
                    finalIntake.setPower(0);
                })
                .addTemporalMarker(0.25, () -> {
                    motorOuttake1.setVelocity(HIGH_VELO+20);
                    motorOuttake2.setVelocity(HIGH_VELO+20);
                })
                .build();

        Trajectory trajectoryyy5 = drive.trajectoryBuilder(trajectoryyy4.end())
                .splineToConstantHeading(new Vector2d(-40, -19), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-21, -19), Math.toRadians(0),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(11, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.05, () -> {
                    motorOuttake1.setVelocity(0);
                    motorOuttake2.setVelocity(0);
                })
                .addTemporalMarker(0.5, () -> {
                    finalIntake.setPower(0.9);
                    motorOuttake1.setVelocity(-300);
                    motorOuttake2.setVelocity(-300);
                    out1.open();
                    out2.open();
                })
                .build();

        Trajectory trajectoryyy6 = drive.trajectoryBuilder(trajectoryyy5.end(), true)
                .lineToSplineHeading(new Pose2d(-54, -4.5, Math.toRadians(7)))
                .addTemporalMarker(0.02, () -> {
                    out1.close();
                    out2.close();
                    finalIntake.setPower(0);
                    motorOuttake1.setVelocity(0);
                    motorOuttake2.setVelocity(0);
                })
                .addTemporalMarker(0.38, () -> {
                    motorOuttake1.setVelocity(HIGH_VELO+20);
                    motorOuttake2.setVelocity(HIGH_VELO+20);
                })
                .build();

        Trajectory trajectoryyy7 = drive.trajectoryBuilder(trajectoryyy6.end())
                .splineToConstantHeading(new Vector2d(-69, 3), Math.toRadians(0))
                .addTemporalMarker(0.1, () -> {
                    motorOuttake1.setVelocity(0);
                    motorOuttake2.setVelocity(0);
                    plug.up();
                })
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
            wob_brat.setServoPositions(0.7);
            if(pipeline.zona == 0)
            {
                drive.followTrajectory(trajectory1);
                outg.close();
                sleep(1000);

                drive.followTrajectory(trajectory2);
                wob_cleste.open();
                sleep(500);
                drive.followTrajectory(trajectory3);
                sleep(15*1000-500);
                drive.followTrajectory(trajectory4);

            }

            else if(pipeline.zona == 1)
            {

                drive.followTrajectory(trajectoryy1);

                outg.close();
                sleep(1000);
                outg.open();

                drive.followTrajectory(trajectoryy2);
                sleep(750);
                wob_brat.down();
                sleep(800);
                wob_cleste.open();
                sleep(500);
                drive.followTrajectory(trajectoryy3);
                sleep(500);
                sleep(3500);
                drive.followTrajectory(trajectoryy4);
                outg.close();
                sleep(1100);
                outg.open();
                drive.followTrajectory(trajectoryy5);
            }

            else if(pipeline.zona == 4)
            {
                drive.followTrajectory(trajectoryyy1);
                outg.close();
                sleep(1000);
                outg.open();

                drive.followTrajectory(trajectoryyy2);
                sleep(400);
                wob_cleste.open();
                sleep(500);
                drive.followTrajectory(trajectoryyy3);
                drive.followTrajectory(trajectoryyy4);
                sleep(250);
                outg.close();
                sleep(950);
                outg.open();

                drive.followTrajectory(trajectoryyy5);
                drive.followTrajectory(trajectoryyy6);
                sleep(250);
                outg.close();
                sleep(950);
                outg.open();
                drive.followTrajectory(trajectoryyy7);

            }

            out1.close();
            out2.close();
            outg.open();
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
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(10,80);

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