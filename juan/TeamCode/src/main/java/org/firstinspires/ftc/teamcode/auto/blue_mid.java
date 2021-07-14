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
public class blue_mid extends LinearOpMode
{

    public static double NEW_P = 65;
    public static double NEW_I = 3.75;
    public static double NEW_D = 0;
    public static double NEW_F = 16.4;
    public double HIGH_VELO = 1560;
    public double POWERSHOT_VELO = 1280;

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



          // ******************************************************************* \\
         // ********************************************************************* \\
        // *****************************  ZERO RINGS  **************************** \\


        //go to the 1st powershot position
        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d(), true)
                .splineTo(new Vector2d(-52, 5), Math.toRadians(180-7))
                .addTemporalMarker(0.5, () -> {
                    finalOuttake.setVelocity(POWERSHOT_VELO);
                })
                .build();


        //go to the 2nd powershot position
        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .strafeTo(new Vector2d(-52, -8))
                .addTemporalMarker(0.1, () -> {
                    outg.open();
                    finalOuttake.setVelocity(POWERSHOT_VELO);
                })
                .build();


        //go to the 3rd powershot position
        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                .strafeTo(new Vector2d(-52, -13.8))
                .addTemporalMarker(0.1, () -> {
                    outg.open();
                    finalOuttake.setVelocity(POWERSHOT_VELO);
                })
                .build();


        //go near the wall and prepare to pick the rings
        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory3.end())
                .lineToSplineHeading(new Pose2d(-120, -28, Math.toRadians(90)))
                .addTemporalMarker(0.01, () -> {
                    finalOuttake.setVelocity(0);
                    plug.down();
                })
                .build();


        //pick up the bounced back rings (if any) and go to the shooting position
        Trajectory trajectory5 = drive.trajectoryBuilder(trajectory4.end())
                .splineToConstantHeading(new Vector2d(-114.5, 13), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-48.5, 0, Math.toRadians(13.5)), Math.toRadians(0))
                .addTemporalMarker(0.01, () -> {
                    out1.open();
                    out2.open();
                    finalOuttake.setVelocity(-400);
                    finalIntake.setPower(0.95);
                })
                .addTemporalMarker(2.5, () -> {
                    out1.close();
                    out2.close();
                    finalIntake.setPower(0);
                    plug.up();
                })
                .addTemporalMarker(2.75, () -> {
                    finalOuttake.setVelocity(HIGH_VELO);
                })
                .build();


        //go and drop the wobble in zone A
        Trajectory trajectory6 = drive.trajectoryBuilder(trajectory5.end(), true)
                //.splineToSplineHeading(new Pose2d(-83, -8, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-73, -25, Math.toRadians(180)), Math.toRadians(0))
                .addTemporalMarker(0.1, () -> {
                    finalOuttake.setVelocity(0);
                })
                .build();


        //park near the red side
        Trajectory trajectory7 = drive.trajectoryBuilder(trajectory6.end())
                .strafeTo(new Vector2d(-85, -10))
                .splineToConstantHeading(new Vector2d(-70, 5), Math.toRadians(0))
                .build();



          // ****************************************************************** \\
         // ******************************************************************** \\
        // *****************************  ONE RING  ***************************** \\


        //go to the 1st powershot position
        Trajectory trajectoryy1 = drive.trajectoryBuilder(new Pose2d(), true)
                .splineTo(new Vector2d(-52, 5), Math.toRadians(180-7))
                .addTemporalMarker(0.5, () -> {
                    finalOuttake.setVelocity(POWERSHOT_VELO);
                })
                .build();


        //go to the 2nd powershot position
        Trajectory trajectoryy2 = drive.trajectoryBuilder(trajectoryy1.end())
                .strafeTo(new Vector2d(-52, -8))
                .addTemporalMarker(0.1, () -> {
                    outg.open();
                    finalOuttake.setVelocity(POWERSHOT_VELO);
                })
                .build();


        //go to the 3rd powershot position
        Trajectory trajectoryy3 = drive.trajectoryBuilder(trajectoryy2.end())
                .strafeTo(new Vector2d(-52, -13.8))
                .addTemporalMarker(0.1, () -> {
                    outg.open();
                    finalOuttake.setVelocity(POWERSHOT_VELO);
                })
                .build();


        //go near the wall and prepare to pick the rings
        Trajectory trajectoryy4 = drive.trajectoryBuilder(trajectoryy3.end())
                .lineToSplineHeading(new Pose2d(-120, -28, Math.toRadians(90)))
                .addTemporalMarker(0.01, () -> {
                    finalOuttake.setVelocity(0);
                    plug.down();
                })
                .build();


        //pick up the bounced back rings (if any) and go to the shooting position
        Trajectory trajectoryy5 = drive.trajectoryBuilder(trajectoryy4.end())
                .splineToConstantHeading(new Vector2d(-114.5, 13), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-48.5, 5, Math.toRadians(15)), Math.toRadians(0))
                .addTemporalMarker(0.01, () -> {
                    out1.open();
                    out2.open();
                    finalOuttake.setVelocity(-400);
                    finalIntake.setPower(0.95);
                })
                .addTemporalMarker(2.5, () -> {
                    out1.close();
                    out2.close();
                    finalIntake.setPower(0);
                    plug.up();
                })
                .addTemporalMarker(2.75, () -> {
                    finalOuttake.setVelocity(HIGH_VELO);
                })
                .build();


        //drop the wobble in zone B
        Trajectory trajectoryy6 = drive.trajectoryBuilder(trajectoryy5.end(), true)
                //.splineToSplineHeading(new Pose2d(-83, -8, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-90, 6, Math.toRadians(180)), Math.toRadians(0))
                .addTemporalMarker(0.1, () -> {
                    finalOuttake.setVelocity(0);
                })
                .build();


        //park near the red side
        Trajectory trajectoryy7 = drive.trajectoryBuilder(trajectoryy6.end())
                .splineToConstantHeading(new Vector2d(-70, 6), Math.toRadians(0))
                .build();




          // ******************************************************************* \\
         // ********************************************************************* \\
        // *****************************  FOUR RINGS  ***************************** \\


        //go to the 1st powershot position
        Trajectory trajectoryyy1 = drive.trajectoryBuilder(new Pose2d(), true)
                .splineTo(new Vector2d(-52, 5), Math.toRadians(180-7))
                .addTemporalMarker(0.5, () -> {
                    finalOuttake.setVelocity(POWERSHOT_VELO);
                })
                .build();


        //go to the 2nd powershot position
        Trajectory trajectoryyy2 = drive.trajectoryBuilder(trajectoryyy1.end())
                .strafeTo(new Vector2d(-52, -8))
                .addTemporalMarker(0.1, () -> {
                    outg.open();
                    finalOuttake.setVelocity(POWERSHOT_VELO);
                })
                .build();


        //go to the 3rd powershot position
        Trajectory trajectoryyy3 = drive.trajectoryBuilder(trajectoryyy2.end())
                .strafeTo(new Vector2d(-52, -13.8))
                .addTemporalMarker(0.1, () -> {
                    outg.open();
                    finalOuttake.setVelocity(POWERSHOT_VELO);
                })
                .build();


        //go near the wall and prepare to pick the rings
        Trajectory trajectoryyy4 = drive.trajectoryBuilder(trajectoryyy3.end())
                .lineToSplineHeading(new Pose2d(-120, -28, Math.toRadians(90)))
                .addTemporalMarker(0.01, () -> {
                    finalOuttake.setVelocity(0);
                    plug.down();
                })
                .build();


        //pick up the bounced back rings (if any) and go to the shooting position
        Trajectory trajectoryyy5 = drive.trajectoryBuilder(trajectoryyy4.end())
                .splineToConstantHeading(new Vector2d(-114.5, 13), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-48.5, 5, Math.toRadians(14)), Math.toRadians(0))
                .addTemporalMarker(0.01, () -> {
                    out1.open();
                    out2.open();
                    finalOuttake.setVelocity(-400);
                    finalIntake.setPower(0.95);
                })
                .addTemporalMarker(2.5, () -> {
                    out1.close();
                    out2.close();
                    finalIntake.setPower(0);
                    plug.up();
                })
                .addTemporalMarker(2.75, () -> {
                    finalOuttake.setVelocity(HIGH_VELO+25);
                })
                .build();


        //go and drop the wobble in zone C
        Trajectory trajectoryyy6 = drive.trajectoryBuilder(trajectoryyy5.end(), true)
                //.splineToSplineHeading(new Pose2d(-83, -8, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-114, -17, Math.toRadians(180)), Math.toRadians(0))
                .addTemporalMarker(0.1, () -> {
                    finalOuttake.setVelocity(0);
                })
                .build();


        //park near the red side
        Trajectory trajectoryyy7 = drive.trajectoryBuilder(trajectoryyy6.end())
                .splineToConstantHeading(new Vector2d(-105, 9), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-69, 8), Math.toRadians(0))
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

                //powershot sequence
                drive.followTrajectory(trajectory1);
                outg.cerc1();
                sleep(500);
                outg.open();
                drive.followTrajectory(trajectory2);
                outg.cerc2();
                sleep(700);
                outg.open();
                drive.followTrajectory(trajectory3);
                outg.close();
                sleep(900);
                outg.open();

                //bounced back rings sequence
                drive.followTrajectory(trajectory4);
                drive.followTrajectory(trajectory5);

                //shoot the rings
                outg.close();
                sleep(900);
                outg.open();

                //wobble
                drive.followTrajectory(trajectory6);
                sleep(500);
                wob_brat.down();
                sleep(700);
                wob_cleste.open();
                sleep(350);
                wob_brat.mid();

                //park
                drive.followTrajectory(trajectory7);
            }

            else if(pipeline.zona == 1)
            {

                //powershot sequence
                drive.followTrajectory(trajectoryy1);
                outg.cerc1();
                sleep(500);
                outg.open();
                drive.followTrajectory(trajectoryy2);
                outg.cerc2();
                sleep(700);
                outg.open();
                drive.followTrajectory(trajectoryy3);
                outg.close();
                sleep(900);
                outg.open();

                //bounced back rings sequence
                drive.followTrajectory(trajectoryy4);
                drive.followTrajectory(trajectoryy5);

                //shoot the rings
                outg.close();
                sleep(900);
                outg.open();

                //wobble
                drive.followTrajectory(trajectoryy6);
                sleep(500);
                wob_brat.down();
                sleep(700);
                wob_cleste.open();
                sleep(350);
                wob_brat.mid();

                //park
                drive.followTrajectory(trajectoryy7);

            }

            else if(pipeline.zona == 4)
            {

                //powershot sequence
                drive.followTrajectory(trajectoryyy1);
                outg.cerc1();
                sleep(500);
                outg.open();
                drive.followTrajectory(trajectoryyy2);
                outg.cerc2();
                sleep(700);
                outg.open();
                drive.followTrajectory(trajectoryyy3);
                outg.close();
                sleep(900);
                outg.open();

                //bounced back rings sequence
                drive.followTrajectory(trajectoryyy4);
                drive.followTrajectory(trajectoryyy5);

                //shoot the rings
                outg.close();
                sleep(900);
                outg.open();

                //wobble
                drive.followTrajectory(trajectoryyy6);
                sleep(500);
                wob_brat.down();
                sleep(700);
                wob_cleste.open();
                sleep(350);
                wob_brat.mid();

                //park
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
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(50,80);

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