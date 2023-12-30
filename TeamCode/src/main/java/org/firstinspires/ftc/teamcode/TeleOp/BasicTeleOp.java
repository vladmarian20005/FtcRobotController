package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Functions.ArmEncoder;
import org.firstinspires.ftc.teamcode.Functions.ClawServos;
import org.firstinspires.ftc.teamcode.Functions.Move;
import org.firstinspires.ftc.teamcode.Functions.PIDController;
import org.firstinspires.ftc.teamcode.Functions.Rotate;
import org.firstinspires.ftc.teamcode.Functions.TopServos;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.advanced.PoseStorage;

@TeleOp(name="Basic TeleOp", group="GAME")
@Disabled
public class BasicTeleOp extends OpMode {
    private DcMotor leftMotor, rightMotor, leftMotorBack, rightMotorBack;
    private DcMotor armMotorLeft, armMotorRight;
    private Servo clawServo, topServo;
    private PIDController controller;
    private Move move;
    private Rotate rotate;
    private ClawServos clawServos;
    private ArmEncoder armEncoder;
    private TopServos topServos;
    private Servo servo;
    private boolean status1;
    int armLevel = 0;
    double integralSum = 0;
    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double Kf = 2;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    String armCurrentDirection = "up";

    private ElapsedTime runtime = new ElapsedTime();

    final double END_GAME = 90.0;
    final double FIFTEEN_SECONDS = 105.0;
    final double FIVE_SECONDS = 115.0;
    boolean secondFifteen = false;
    boolean secondEnd = false;
    boolean secondFive = false;
    Gamepad.RumbleEffect customRumbleEffectFive, customRumbleEffectFifteen, customRumbleEffectEnd;
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    @Override
    public void init() {
        customRumbleEffectFive = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 500)  //  Rumble both motors 100% for 500 mSec
                .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
                .addStep(1.0, 1.0, 500)  //  Rumble both motors 100% for 500 mSec
                .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
                .addStep(1.0, 1.0, 500)  //  Rumble both motors 100% for 500 mSec
                .build();

        customRumbleEffectFifteen = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 600)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
                .addStep(1.0, 1.0, 600)  //  Rumble left motor 100% for 250 mSec
                .build();

        customRumbleEffectEnd = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 700)  //  Rumble right motor 100% for 500 mSec
                .build();


        leftMotor = hardwareMap.dcMotor.get("FL");
        rightMotor = hardwareMap.dcMotor.get("FR");
        leftMotorBack = hardwareMap.dcMotor.get("BL");
        rightMotorBack = hardwareMap.dcMotor.get("BR");
        armMotorLeft = hardwareMap.dcMotor.get("AML");
        armMotorRight = hardwareMap.dcMotor.get("AMR");
        topServo = hardwareMap.servo.get("TS");
        clawServo = hardwareMap.servo.get("CS");
        move = new Move(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        rotate = new Rotate(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        clawServos = new ClawServos(clawServo);
        armEncoder = new ArmEncoder(armMotorLeft, armMotorRight);
        topServos = new TopServos(topServo);
        controller = new PIDController(armMotorLeft, armMotorRight);


        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);

        armMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        servosUp(topServo);
    }

    @Override
    public void loop() {
        //Watch the runtime timer, and run the custom rumble when we hit half-time.
        //Make sure we only signal once by setting "secondHalf" flag to prevent further rumbles.
        if ((runtime.seconds() > FIFTEEN_SECONDS) && !secondFifteen)  {
            gamepad1.runRumbleEffect(customRumbleEffectFifteen);
            gamepad2.runRumbleEffect(customRumbleEffectFifteen);
            secondFifteen =true;
        }
        else if ((runtime.seconds() > END_GAME) && !secondEnd)  {
            gamepad1.runRumbleEffect(customRumbleEffectEnd);
            gamepad2.runRumbleEffect(customRumbleEffectEnd);
            secondEnd =true;
        }
        else if ((runtime.seconds() > FIVE_SECONDS) && !secondFive)  {
            gamepad1.runRumbleEffect(customRumbleEffectFive);
            gamepad2.runRumbleEffect(customRumbleEffectFive);
            secondFive =true;
        }


        Pose2d poseEstimate = drive.getPoseEstimate();
        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x
        ).rotated(-poseEstimate.getHeading());

        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -gamepad1.right_stick_x
                )
        );
        drive.update();


        if(gamepad1.y)
        {
            controller.goTo(1000, 1000);
        }
        if(gamepad2.b)
        {
            telemetry.addData("x", clawServo.getDirection());
            telemetry.update();
            clawServos.SwitchAndWait(1,getRuntime());
        }
        if (gamepad1.x) {
            clawServos.SwitchAndWait(1, getRuntime());

        }
        if(gamepad1.right_bumper)
        {
            drive.setPoseEstimate(PoseStorage.currentPose);
        }


        if (gamepad2.dpad_up) // Arm Up
        {
            armCurrentDirection = "up";
            armEncoder.goTo(2800, 2800);
            while(armMotorLeft.isBusy() && armMotorRight.isBusy())
            {
                try
                {
                    continue;
                }
                catch(Exception e)
                {
                    armEncoder.goTo(0,0);
                }
            }
            servosDown(topServo);


        }
        else if (gamepad2.dpad_down) //Arm Down
        {
            armCurrentDirection = "down";
            // closeServo(leftServo);
            servosUp(topServo);
            armEncoder.goTo(0, 0);
            closeServo(clawServo);

        }
        if (gamepad1.dpad_up) // Arm Up
        {
            armCurrentDirection = "up";
            armEncoder.goTo(1000, 1000);

            servosDown(topServo);
            armEncoder.goTo(3000, 3000);
        }
        else if(gamepad2.dpad_left) // Arm level 1
        {
            armCurrentDirection = "up";
            armEncoder.goTo(1300, 1300);

            servosDown(topServo);

        }
        else if(gamepad2.dpad_right) // Level 2
        {
            armCurrentDirection = "up";
            armEncoder.goTo(1300, 1300);

            servosDown(topServo);
            armEncoder.goTo(2000, 2000);

        }
        else if (gamepad2.y) {
            armCurrentDirection = "up";
            armEncoder.goTo(100, 100);
        }
        else if (gamepad1.dpad_down) //Arm Down
        {
            armCurrentDirection = "down";
            // closeServo(leftServo);
            servosUp(topServo);

        }

     //   closeServo(clawServo);
        if (armCurrentDirection.equals("down")) {
            armMotorLeft.setPower(0);
            armMotorRight.setPower(0);
            armMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }



    private void openServo(Servo _LS)
    {
        _LS.setPosition(1);
    }
    private void closeServo(Servo _LS)
    {
        _LS.setPosition(0);
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
