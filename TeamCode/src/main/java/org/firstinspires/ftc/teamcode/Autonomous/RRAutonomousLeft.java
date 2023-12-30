package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Functions.ArmEncoder;
import org.firstinspires.ftc.teamcode.Functions.ClawServos;
import org.firstinspires.ftc.teamcode.Functions.TopServos;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class RRAutonomousLeft extends LinearOpMode {
    private SleeveDetection sleeveDetection;
    private OpenCvCamera camera;
    private Servo clawServo, topServo;
    private ArmEncoder armEncoder;
    private DcMotor armMotorLeft, armMotorRight;
    private String webcamName = "Webcam";
    private String currentPosition;
    private ClawServos clawServos;
    private TopServos topServos;
    @Override
    public void runOpMode() throws InterruptedException {
        armMotorLeft = hardwareMap.dcMotor.get("AML");
        armMotorRight = hardwareMap.dcMotor.get("AMR");
        armEncoder = new ArmEncoder(armMotorLeft, armMotorRight);
        clawServo =  hardwareMap.servo.get("CS");
        topServo = hardwareMap.servo.get("TS");
        topServos = new TopServos(topServo);
        clawServos = new ClawServos(clawServo);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection();
        camera.setPipeline(sleeveDetection);
        drive.setPoseEstimate(new Pose2d(-61.70, 33.0, Math.toRadians(180.0)));
        armMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TrajectorySequence advancedTrajTry, forwardToPole, stackTraj1, stackTraj2, takeConeStack1, takeConeStack2, backFromStack1, backFromStack2, poleTraj1, poleTraj2, forwardToPoleStack1, forwardToPoleStack2;
        int ticks = 400;

        advancedTrajTry = drive.trajectorySequenceBuilder(new Pose2d(-61.70, 33.0, Math.toRadians(180.0)))
                .strafeLeft(3.0)
                .lineToConstantHeading(new Vector2d(-8.0, 33.0)) // -12 -33

                .addTemporalMarker(0.20, () ->
                {
                    armEncoder.goTo(2800, 2800);
                })
                .lineToConstantHeading(new Vector2d(-12,33.0))
                .strafeLeft(11.0)
                .build();

        forwardToPole = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ advancedTrajTry.end())
                .back(2.0)
                .build();

        stackTraj1 = drive.trajectorySequenceBuilder(/*new Pose2d(-8.5, -25.83, Math.toRadians(0.0))*/ forwardToPole.end())
                .forward(6.0)
                .lineToSplineHeading(new Pose2d(-10.0, 58.0, Math.toRadians(-90.0)))
                .addTemporalMarker(1, () ->
                {
                    armEncoder.goTo(350, 350);
                })
                .build();

        takeConeStack1 = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ stackTraj1.end())
                .back(2.0)
                .build();

        backFromStack1 = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ takeConeStack1.end())
                .forward(7.0)
                .build();

        poleTraj1 = drive.trajectorySequenceBuilder(backFromStack1.end())
                .lineToSplineHeading(new Pose2d(-11.0, 22.5, Math.toRadians(180.0) +1e-6)) //13 si 26 cu minus
                .build();

        forwardToPoleStack1 = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ poleTraj1.end())
                .back(4.0)
                .build();

        stackTraj2 = drive.trajectorySequenceBuilder(/*new Pose2d(-8.5, -25.83, Math.toRadians(0.0))*/ forwardToPoleStack1.end())
                .forward(5.0)
                .lineToSplineHeading(new Pose2d(-7.0, 56.0, Math.toRadians(-90.0)))
                .addTemporalMarker(1, () ->
                {
                    armEncoder.goTo(250, 250);
                })
                .build();

        takeConeStack2 = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ stackTraj2.end())
                .back(2.0)
                .build();


        backFromStack2 = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ takeConeStack2.end())
                .forward(7.0)
                .build();

        poleTraj2 = drive.trajectorySequenceBuilder(backFromStack2.end())
                .lineToSplineHeading(new Pose2d(-9.0, 22.0, Math.toRadians(180.0) +1e-6)) //15 si 27
                .build();

        forwardToPoleStack2 = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ poleTraj2.end())
                .back(4.5)
                .build();

        TrajectorySequence stackTraj3 = drive.trajectorySequenceBuilder(/*new Pose2d(-8.5, -25.83, Math.toRadians(0.0))*/ forwardToPoleStack2.end())
                .forward(5.5)
                .lineToSplineHeading(new Pose2d(-3.0, 56.0, Math.toRadians(-90.0)))
                .addTemporalMarker(1, () ->
                {
                    armEncoder.goTo(170, 170);
                })
                .build();

        TrajectorySequence takeConeStack3 = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ stackTraj3.end())
                .back(1.8)
                .build();


        TrajectorySequence backFromStack3 = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ takeConeStack3.end())
                .forward(7.0)
                .build();

        TrajectorySequence poleTraj3 = drive.trajectorySequenceBuilder(backFromStack3.end())
                .lineToSplineHeading(new Pose2d(-9.1, 21.0, Math.toRadians(180.0) +1e-6)) //15 si 27
                .build();

        TrajectorySequence forwardToPoleStack3 = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ poleTraj3.end())
                .back(5.0)
                .build();

        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ forwardToPoleStack3.end())
                .strafeLeft(11.5)
                .build();

        TrajectorySequence centerPark = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ forwardToPoleStack3.end())
                .strafeRight(15.0)
                .build();

        TrajectorySequence rightPark = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ forwardToPoleStack3.end())
                .strafeRight(32.0)
                .build();

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(432,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        while (!isStarted()) {
            telemetry.addData("ROTATION: ", sleeveDetection.getPosition());
            telemetry.update();
        }

        waitForStart();
        currentPosition = String.valueOf(sleeveDetection.getPosition());

        if(isStopRequested()) return;

        if(currentPosition == "LEFT")
        {
            servosUp(topServo);
            closeServo(clawServo);
            sleep(250);
            armEncoder.goTo(50, 50);

            drive.followTrajectorySequence(advancedTrajTry);
            drive.followTrajectorySequence(forwardToPole);
            servosUp(topServo);

            openServo(clawServo);
            sleep(100);
            armEncoder.goTo(0,0);

            //Con preloaded


            drive.followTrajectorySequence(stackTraj1);

            sleep(100);

            drive.followTrajectorySequence(takeConeStack1);

            sleep(200);
            closeServo(clawServo);
            sleep(250);

            armEncoder.goTo(2800, 2800);
            sleep(200);
//            drive.followTrajectorySequence(backFromStack1);

            drive.followTrajectorySequence(poleTraj1);
            drive.followTrajectorySequence(forwardToPoleStack1);
            servosUp(topServo);

            sleep(200);
            openServo(clawServo);
            sleep(100);
            armEncoder.goTo(0,0);

            //Con 1 Stack

            drive.followTrajectorySequence(stackTraj2);

            sleep(100);

            drive.followTrajectorySequence(takeConeStack2);

            sleep(200);
            closeServo(clawServo);
            sleep(250);

            armEncoder.goTo(2800, 2800);
            sleep(200);
//            drive.followTrajectorySequence(backFromStack2);

            drive.followTrajectorySequence(poleTraj2);
            drive.followTrajectorySequence(forwardToPoleStack2);
            servosUp(topServo);

            sleep(200);
            openServo(clawServo);
            sleep(100);
            armEncoder.goTo(0,0);

            //Con 2 Stack

            drive.followTrajectorySequence(stackTraj3);

            sleep(100);

            drive.followTrajectorySequence(takeConeStack3);

            sleep(200);
            closeServo(clawServo);
            sleep(250);

            armEncoder.goTo(2800, 2800);
            sleep(200);
//            drive.followTrajectorySequence(backFromStack3);

            drive.followTrajectorySequence(poleTraj3);
            drive.followTrajectorySequence(forwardToPoleStack3);
            servosUp(topServo);

            sleep(200);
            openServo(clawServo);
            sleep(100);
            armEncoder.goTo(0,0);

            //Cone 3 Stack

            drive.followTrajectorySequence(rightPark);

        }
        else if(currentPosition=="CENTER")
        {
            servosUp(topServo);
          closeServo(clawServo);
            sleep(250);
            armEncoder.goTo(50, 50);

            drive.followTrajectorySequence(advancedTrajTry);
            drive.followTrajectorySequence(forwardToPole);
            servosUp(topServo);

            openServo(clawServo);
            sleep(100);
            armEncoder.goTo(0,0);

            //Con preloaded


            drive.followTrajectorySequence(stackTraj1);

            sleep(100);

            drive.followTrajectorySequence(takeConeStack1);

            sleep(200);
            closeServo(clawServo);
            sleep(250);

            armEncoder.goTo(2800, 2800);

//            drive.followTrajectorySequence(backFromStack1);

            drive.followTrajectorySequence(poleTraj1);
            drive.followTrajectorySequence(forwardToPoleStack1);
            servosUp(topServo);

            sleep(200);
            openServo(clawServo);
            sleep(100);
            armEncoder.goTo(0,0);

            //Con 1 Stack

            drive.followTrajectorySequence(stackTraj2);

            sleep(100);

            drive.followTrajectorySequence(takeConeStack2);

            sleep(200);
            closeServo(clawServo);
            sleep(250);

            armEncoder.goTo(2800, 2800);

//            drive.followTrajectorySequence(backFromStack2);

            drive.followTrajectorySequence(poleTraj2);
            drive.followTrajectorySequence(forwardToPoleStack2);
            servosUp(topServo);

            sleep(200);
            openServo(clawServo);
            sleep(100);
            armEncoder.goTo(0,0);

            //Con 2 Stack

            drive.followTrajectorySequence(stackTraj3);

            sleep(100);

            drive.followTrajectorySequence(takeConeStack3);

            sleep(200);
            closeServo(clawServo);
            sleep(250);

            armEncoder.goTo(2800, 2800);

//            drive.followTrajectorySequence(backFromStack3);

            drive.followTrajectorySequence(poleTraj3);
            drive.followTrajectorySequence(forwardToPoleStack3);
            servosUp(topServo);

            sleep(200);
            openServo(clawServo);
            sleep(100);
            armEncoder.goTo(0,0);

            //Cone 3 Stack

            drive.followTrajectorySequence(centerPark);
        }
        else if(currentPosition=="RIGHT")
        {
            servosUp(topServo);
            closeServo(clawServo);
            sleep(250);
            armEncoder.goTo(50, 50);

            drive.followTrajectorySequence(advancedTrajTry);
            drive.followTrajectorySequence(forwardToPole);
            servosUp(topServo);

            openServo(clawServo);
            sleep(100);
            armEncoder.goTo(0,0);

            //Con preloaded


            drive.followTrajectorySequence(stackTraj1);

            sleep(100);

            drive.followTrajectorySequence(takeConeStack1);

            sleep(200);
            closeServo(clawServo);
            sleep(250);

            armEncoder.goTo(2800, 2800);

//            drive.followTrajectorySequence(backFromStack1);

            drive.followTrajectorySequence(poleTraj1);
            drive.followTrajectorySequence(forwardToPoleStack1);
            servosUp(topServo);

            sleep(200);
            openServo(clawServo);
            sleep(100);
            armEncoder.goTo(0,0);

            //Con 1 Stack

            drive.followTrajectorySequence(stackTraj2);

            sleep(100);

            drive.followTrajectorySequence(takeConeStack2);

            sleep(200);
            closeServo(clawServo);
            sleep(250);

            armEncoder.goTo(2800, 2800);

//            drive.followTrajectorySequence(backFromStack2);

            drive.followTrajectorySequence(poleTraj2);
            drive.followTrajectorySequence(forwardToPoleStack2);
            servosUp(topServo);

            sleep(200);
            openServo(clawServo);
            sleep(100);
            armEncoder.goTo(0,0);

            //Con 2 Stack

            drive.followTrajectorySequence(stackTraj3);

            sleep(100);

            drive.followTrajectorySequence(takeConeStack3);

            sleep(200);
            closeServo(clawServo);
            sleep(250);

            armEncoder.goTo(2800, 2800);

//            drive.followTrajectorySequence(backFromStack3);

            drive.followTrajectorySequence(poleTraj3);
            drive.followTrajectorySequence(forwardToPoleStack3);
            servosUp(topServo);

            sleep(200);
            openServo(clawServo);
            sleep(100);
            armEncoder.goTo(0,0);

            //Cone 3 Stack

            drive.followTrajectorySequence(leftPark);
        }
    }
    private void openServo(Servo _LS)
    {
        _LS.setPosition(0);
    }
    private void closeServo(Servo _LS)
    {
        _LS.setPosition(1);
    }
    private void servosUp(Servo topLeftServo)
    {
        topLeftServo.setPosition(0);
        //topRightServo.setPosition(0);
    }
    private void servosDown(Servo topLeftServo)
    {
        topLeftServo.setPosition(1);
        //topRightServo.setPosition(1);
    }
}
