package org.firstinspires.ftc.teamcode.Orig.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Functions.ArmEncoder;
import org.firstinspires.ftc.teamcode.Functions.GamepadCalc;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.advanced.PoseStorage;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "AutoYellowGreen", group = "ORIG")
public class RRAutonomousYellowGreen extends LinearOpMode {
    //Declare webcam
    private String webcamName = "Webcam";
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private static final String TFOD_MODEL_FILE =  "/sdcard/FIRST/tflitemodels/prop.tflite";
    private static final String[] LABELS = {
            "Left", "Center", "Right"
    };
    private TfodProcessor tfod;

    //Declare motors
    private DcMotor leftMotor, rightMotor, leftMotorBack, rightMotorBack;
    private DcMotor armMotorLeft, armMotorRight;

    //Declare classes
    private ArmEncoder controller;

    //Other declarations
    String detection = "";
    GamepadCalc gamepadCalc;
    VisionPortal visionPortal;

    public void runOpMode() throws InterruptedException {
        //init motors chasis
        leftMotor = hardwareMap.dcMotor.get("FL");
        rightMotor = hardwareMap.dcMotor.get("FR");
        leftMotorBack = hardwareMap.dcMotor.get("BL");
        rightMotorBack = hardwareMap.dcMotor.get("BR");

        //init arm motors
        armMotorLeft = hardwareMap.dcMotor.get("SL");
        armMotorRight = hardwareMap.dcMotor.get("SR");

        //init classes
        controller = new ArmEncoder(armMotorLeft, armMotorRight);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        gamepadCalc = new GamepadCalc(this);

        //chasis motors mode
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //RR init
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);


        while (!isStarted()) {
            telemetryTfod();
            // Push telemetry to the Driver Station.
            telemetry.update();
        }
        visionPortal.close();
        waitForStart();
        if(opModeIsActive()) {

        }
    }

    //Tfod function
    private void telemetryTfod() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        if (!currentRecognitions.isEmpty()) {
            // Assuming the model returns only one classification result for the whole image
            Recognition classification = currentRecognitions.get(0);

            // Get the image width from the recognition object (assuming it's the same as the camera resolution you set)
            int imageWidth = 640; // For example, if your camera resolution is set to 640x480

            // Calculate the midpoint of the detected object
            float objectMidpoint = (classification.getLeft() + classification.getRight()) / 2;

            // Determine the position of the object
            String position;
            if (objectMidpoint < imageWidth / 3) {
                position = "Left";
                detection = "l";
            } else if (objectMidpoint < (2 * imageWidth / 3)) {
                position = "Center";
                detection = "c";
            } else {
                position = "Right";
                detection = "r";
            }

            telemetry.addData("Classified Image", "%s (%.0f %% Conf.)",
                    classification.getLabel(), classification.getConfidence() * 100);
            telemetry.addData("Position", position);
        } else {
            telemetry.addData("No Classification", "");
        }
    }  // end method telemetryTfod()
}
