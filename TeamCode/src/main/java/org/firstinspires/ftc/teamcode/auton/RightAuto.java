/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class RightAuto extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int Left = 1;
    int Middle = 2;
    int Right = 3;

    AprilTagDetection tagOfInterest = null;



    @Override
    public void runOpMode()
    {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        DcMotor Lr = hardwareMap.dcMotor.get("Lr");
        DcMotor Ll = hardwareMap.dcMotor.get("Ll");
        Servo Geoff = hardwareMap.servo.get("Geoff");

        Lr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Ll.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //double IntegralSum = 0;
        //double kp = 0.01;
        //double ki = 0.0001;
        //double kd = 0.0085;

        //double lastError = 0;
        //double target = 0;
        double lastClick = 0;

        int cones = 1;
        double servoPos = 0.5;
        double EncoderSlidesFix = 0;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        //set starting coordinates here
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        //Trajectories Here!!
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .splineToConstantHeading(
                        new Vector2d(11,19), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .forward(
                        38.5,
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end().plus(new Pose2d(0, 0, Math.toRadians(-93))), false)
                .forward(
                        42,
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end(), false)
                .forward(
                        9.5,
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end(), false)
                .back(
                        51.5,
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end().plus(new Pose2d(0, 0, Math.toRadians(50))), false)
                .forward(
                        6,
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory traj6 = drive.trajectoryBuilder(traj5.end(), false)
                .forward(
                        5,
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory traj7 = drive.trajectoryBuilder(traj6.end(), false)
                .back(
                        12,
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == Left ||tag.id == Middle || tag.id == Right)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */

        Geoff.setPosition(0.5);

        if(tagOfInterest == null || tagOfInterest.id == Left)
        {
            drive.followTrajectory(traj1);
            drive.turn(Math.toRadians(-93));
            drive.followTrajectory(traj2);

            //insert code here to pick up cone from stack
            moveLift(1700, Lr, Ll, false);
            drive.followTrajectory(traj3);
            moveLift(500, Lr, Ll, false);
            Geoff.setPosition(0.8);
            sleep(750);
            moveLift(1500, Lr, Ll, false);

            drive.followTrajectory(traj4);
            drive.turn(Math.toRadians(50));
            drive.followTrajectory(traj5);
            moveLift(2950, Lr, Ll, true);
            drive.followTrajectory(traj6);
            Geoff.setPosition(0.5);
            sleep(500);
            drive.followTrajectory(traj7);


        }
        else if(tagOfInterest.id == Middle)
        {
            drive.followTrajectory(traj1);
            drive.turn(Math.toRadians(-93));
            drive.followTrajectory(traj2);

            //insert code here to pick up cone from stack
            moveLift(1700, Lr, Ll, false);
            drive.followTrajectory(traj3);
            moveLift(500, Lr, Ll, false);
            Geoff.setPosition(0.8);
            sleep(750);
            moveLift(1500, Lr, Ll, false);

            drive.followTrajectory(traj4);
            drive.turn(Math.toRadians(50));
            drive.followTrajectory(traj5);
            moveLift(2950, Lr, Ll, true);
            drive.followTrajectory(traj6);
            Geoff.setPosition(0.5);
            sleep(500);
            drive.followTrajectory(traj7);
        }
        else
        {
            drive.followTrajectory(traj1);
            drive.turn(Math.toRadians(-93));
            drive.followTrajectory(traj2);

            //insert code here to pick up cone from stack
            moveLift(1700, Lr, Ll, false);
            drive.followTrajectory(traj3);
            moveLift(500, Lr, Ll, false);
            Geoff.setPosition(0.8);
            sleep(750);
            moveLift(1500, Lr, Ll, false);

            drive.followTrajectory(traj4);
            drive.turn(Math.toRadians(50));
            drive.followTrajectory(traj5);
            moveLift(2950, Lr, Ll, true);
            drive.followTrajectory(traj6);
            Geoff.setPosition(0.5);
            sleep(500);
            drive.followTrajectory(traj7);
        }

    }


    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
//        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    void moveLift(double target, DcMotor Lr, DcMotor Ll, boolean hold) {
        double IntegralSum =0;
        double lastError = 0;

        double kp = 0.01;
        double ki = 0.0001;
        double kd = 0.0085;

        ElapsedTime et = new ElapsedTime();

        while((Ll.getCurrentPosition() <= (target - 5)) || (Ll.getCurrentPosition() > (target + 5))) {
            telemetry.addData("Target: ", target);
            telemetry.addData("CurrentPosition: ", Ll.getCurrentPosition());
            telemetry.update();

            if(et.seconds() > 4){
                break;
            }

            double error = target - Ll.getCurrentPosition();

            IntegralSum += error;

            if (error < 1) {
                IntegralSum = 0;
            }
            double derivative = (error - lastError);
            lastError = error;

            double Power = (error * kp) + (IntegralSum * ki) + (derivative * kd);

            if (Power > 1) {
                Power = 1;
            } else if (Power < -1) {
                Power = -1;
            }
            Ll.setPower(Power);
            Lr.setPower(-Power);
            if(hold){
                ElapsedTime et2 = new ElapsedTime();
                while(et.seconds() < 3) {
                    Ll.setPower(0.25);
                    Lr.setPower(-0.25);
                }
            }
        }
    }

}
