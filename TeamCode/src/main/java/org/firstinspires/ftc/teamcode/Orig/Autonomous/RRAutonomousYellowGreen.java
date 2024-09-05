package org.firstinspires.ftc.teamcode.Orig.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Functions.ArmEncoder;
import org.firstinspires.ftc.teamcode.Functions.GamepadCalc;
import org.firstinspires.ftc.teamcode.Orig.TeleOp.Functions.ClawArmEncoder;
import org.firstinspires.ftc.teamcode.Orig.TeleOp.Functions.ClawServos;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "AutoYellowGreen", group = "ORIG")
public class RRAutonomousYellowGreen extends LinearOpMode {

    //Declare motors
    private DcMotor leftMotor, rightMotor, leftMotorBack, rightMotorBack;
    private DcMotor armMotorLeft, armMotorRight;
    private DcMotor clawMotor;

    //Declare servos
    private Servo closeClaw, rotateClaw;

    //Declare classes
    private ArmEncoder controller;
    private ClawArmEncoder clawArmEncoder;
    private ClawServos clawServos;

    //Other declarations
    GamepadCalc gamepadCalc;

    public void runOpMode() throws InterruptedException {
        //init motors chasis
        leftMotor = hardwareMap.dcMotor.get("FL");
        rightMotor = hardwareMap.dcMotor.get("FR");
        leftMotorBack = hardwareMap.dcMotor.get("BL");
        rightMotorBack = hardwareMap.dcMotor.get("BR");

        //init arm motors
        armMotorLeft = hardwareMap.dcMotor.get("SL");
        armMotorRight = hardwareMap.dcMotor.get("SR");

        //claw motor
        clawMotor = hardwareMap.dcMotor.get("CM");

        //init servos
        closeClaw = hardwareMap.servo.get("CS");
        rotateClaw = hardwareMap.servo.get("RS");

        //init classes
        controller = new ArmEncoder(armMotorLeft, armMotorRight);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        gamepadCalc = new GamepadCalc(this);
        clawServos = new ClawServos(closeClaw, rotateClaw);

        //chasis motors mode
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //RR init
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);


        while (!isStarted()) {
            //telemetryTfod();
            // Push telemetry to the Driver Station.
            telemetry.update();
        }
       // visionPortal.close();
        waitForStart();
        TrajectorySequence trajectory1RL = drive.trajectorySequenceBuilder(new Pose2d(12, 62, Math.toRadians(-90)))
                .addDisplacementMarker(() -> {
                    clawArmEncoder.goTo(20);
                    clawServos.RotateCenter();
                                        
                })

    }

}
